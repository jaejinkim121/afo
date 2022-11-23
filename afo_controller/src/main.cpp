#include "../include/main.hpp"

void pathPlanner(){
    auto time = high_resolution_clock::now();
    auto currentTimeGap = duration_cast<microseconds>(time-timeIC);
    auto eventTimeGap = duration_cast<microseconds>(timeOFO - timeIC);
    double currentCyclePercentage = currentTimegap / eventTimeGap * 0.12;

    if (currentCyclePercentage < 0.35){
        plantarPosition = 0;
        plantarTorque = 0;
        dorsiPosition = 0;
        dorsiTorque = 0;
    }
    else if (currentCyclePercentage < 0.5){
        plantarPosition = 0;
        plantarTorque = 3 * (currentCyclePercentage - 0.35) ^ 2 - 2 * (currentCyclePercentage - 0.35) ^ 3;
        dorsiPosition = 0;
        dorsiTorque = 0;
    }
    else if (currentCyclePercentage < 0.6){
        plantarPosition = 0;
        plantarTorque = 1 - 3 * (currentCyclePercentage - 0.5) ^ 2 + 2 * (currentCyclePercentage - 0.5) ^ 3;
        dorsiPosition = 0;
        dorsiTorque = 0;
    }
    else if (currentCyclePercentage < 0.7){
        plantarPosition = 0;
        plantarTorque = 0;
        dorsiPosition = currentCyclePercentage - 0.7;
        dorsiTorque = 0;
    }
    else if (currentCyclePercentage < 0.75){
        plantarPosition = 0;
        plantarTorque = 0;
        dorsiPosition = 2.4 - 2 * currentCyclePercentage;
        dorsiTorque = 0;
    }
    else {
        plantarPosition = 0;
        plantarTorque = 0;
        dorsiPosition = 0;
        dorsiTorque = 0;
    }

    return;
}

void callbackGaitPhase(const std_msgs::Bool::ConstPtr& msg){
    if (msg->data == true){     
        timeIC = high_resolution_clock::now();
    }
    else {
        timeOFO = high_resolution_clock::now();
    }
    
    return;
}

void worker()
{
    bool rtSuccess = true;
    for(const auto & master: configurator->getMasters())
    {
        rtSuccess &= master->setRealtimePriority(99);
    }

    bool maxonEnabledAfterStartup = false;
    /*
    ** The communication update loop.
    ** This loop is supposed to be executed at a constant rate.
    ** The EthercatMaster::update function incorporates a mechanism
    ** to create a constant rate.
     */
    while(!abrt)
    {
        /*
        ** Update each master.
        ** This sends tha last staged commands and reads the latest readings over EtherCAT.
        ** The StandaloneEnforceRate update mode is used.
        ** This means that average update rate will be close to the target rate (if possible).
         */
        for(const auto & master: configurator->getMasters() )
        {
            master->update(ecat_master::UpdateMode::StandaloneEnforceRate); // TODO fix the rate compensation (Elmo reliability problem)!!
        }

        for(const auto & slave:configurator->getSlaves())
        {  
            // Keep constant update rate
            // auto start_time = std::chrono::steady_clock::now();

            std::shared_ptr<maxon::Maxon> maxon_slave_ptr = std::dynamic_pointer_cast<maxon::Maxon>(slave);

            if (!maxonEnabledAfterStartup)
            {
                // Set maxons to operation enabled state, do not block the call!
                maxon_slave_ptr->setDriveStateViaPdo(maxon::DriveState::OperationEnabled, false);
            }

            // set commands if we can
            if (maxon_slave_ptr->lastPdoStateChangeSuccessful() &&
                    maxon_slave_ptr->getReading().getDriveState() == maxon::DriveState::OperationEnabled)
            {
                // CONTROL LOOP MAIN BODY
                if (slave->getName() == "Plantar"){
                    auto reading = maxon_slave_ptr->getReading();
                    // Find and switch correct control mode for current command.

                    // 

                    maxon::Command command;
                    command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
                    command.setTargetPosition(reading.getActualPosition() + 10);
                    command.setTargetTorque(-0.5);
                    maxon_slave_ptr->stageCommand(command);
                }
                else if (slave->getName() == "Dorsi"){

                }
                else {
                    std::cout << slave->getName() << " is not our target device" << std::endl;
                }
            }
            else
            {
                MELO_WARN_STREAM("Maxon '" << maxon_slave_ptr->getName()
                                                                        << "': " << maxon_slave_ptr->getReading().getDriveState());
            }

            // Constant update rate
            // std::this_thread::sleep_until(start_time + std::chrono::milliseconds(1));
        }
        maxonEnabledAfterStartup = true;
    }
}

/*
** Handle the interrupt signal.
** This is the shutdown routine.
** Note: This logic is executed in a thread separated from the communication update!
 */
void signal_handler(int sig)
{
    /*
    ** Pre shutdown procedure.
    ** The devices execute procedures (e.g. state changes) that are necessary for a
    ** proper shutdown and that must be done with PDO communication.
    ** The communication update loop (i.e. PDO loop) continues to run!
    ** You might thus want to implement some logic that stages zero torque / velocity commands
    ** or simliar safety measures at this point using e.g. atomic variables and checking them
    ** in the communication update loop.
     */
    for(const auto & master: configurator->getMasters())
    {
        master->preShutdown();
    }

    // stop the PDO communication at the next update of the communication loop
    abrt = true;
    worker_thread->join();

    /*
    ** Completely halt the EtherCAT communication.
    ** No online communication is possible afterwards, including SDOs.
     */
    for(const auto & master: configurator->getMasters())
    {
        master->shutdown();
    }

    // Exit this executable
    std::cout << "Shutdown" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    exit(0);
}


/*
** Program entry.
** Pass the path to the setup.yaml file as first command line argument.
 */
int main(int argc, char**argv)
{
    ros::init(argc, argv, "afo_controller");
    ros::NodeHandle n;
    int rr;
    n.getParam("/rr", rr);
    ros::Rate loop_rate(rr);
    ros::Subscriber afo_gaitPhase = n.subscribe<std_msgs::Bool>("/afo_predictor/gaitEvent", 1, callbackGaitPhase);

    std::signal(SIGINT, signal_handler);

    configurator = std::make_shared<EthercatDeviceConfigurator>(argv[1]);

    /*
    ** Start all masters.
    ** There is exactly one bus per master which is also started.
    ** All online (i.e. SDO) configuration is done during this call.
    ** The EtherCAT interface is active afterwards, all drives are in Operational
    ** EtherCAT state and PDO communication may begin.
     */
    for(auto & master: configurator->getMasters())
    {
        if(!master->startup())
        {
            std::cerr << "Startup not successful." << std::endl;
            return EXIT_FAILURE;
        }
    }

    // Start the PDO loop in a new thread.
    worker_thread = std::make_unique<std::thread>(&worker);

    /*
    ** Wait for a few PDO cycles to pass.
    ** Set anydrives into to ControlOp state (internal state machine, not EtherCAT states)
     */

    std::cout << "Startup finished" << std::endl;

    // nothing further to do in this thread.
    pause();
}