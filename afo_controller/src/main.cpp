#include "../include/main.hpp"

void pathPlannerPlantarflexion(Reading reading){
    auto time = high_resolution_clock::now();
    auto currentTimeGap = duration_cast<microseconds>(time-timeIC);
    auto eventTimeGap = duration_cast<microseconds>(timeOFO - timeIC);
    double currentCyclePercentage = currentTimegap / eventTimeGap * 0.12;

    // Dummy variable to simplify formulation.
    double t;   

    // Before actuation
    if (currentCyclePercentage < startTime){
        plantarPosition = plantarNeutralPosition;
        plantarTorque = 0;
        plantarMode = maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode;
    }
    // Start plantarflexion, torque increasing
    else if (currentCyclePercentage < startTime + onTime * upTimeRatio){
        t = currentCyclePercentage - startTime;
        
        plantarPosition = 0;
        // Acceleration
        if (t < 0.5 * upTimeRatio * onTime){
            plantarTorque = 0.5 * acc * t^2;
        }
        // Deceleration
        else{
            plantarTorque = 1 - 0.5 * acc * (t - onTime * upTimeRatio) ^ 2;
        }
        plantarMode = maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode;     
    }
    // Still Plantarflexion, torque decreasing
    else if (currentCyclePercentage < endTime){
        t = currentCyclePercentage - startTime - onTime * upTimeRatio;

        plantarPosition = 0;

        // Acceleration
        if (t < 0.5 * (1 - upTimeRatio) * onTime) {
            plantarTorque = 1 - 0.5 * acc * upTimeRatio / (1 - upTimeRatio) * t ^ 2;
        }
        // Deceleration
        else {
            plantarTorque = 0.5 * acc * upTimeRatio / (1 - upTimeRatio) * (t - onTime * (1 - upTimeRatio)) ^ 2;
        }
        plantarMode = maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode;
    }
    // After end of plantarflexion
    else {
        plantarPosition = plantarNeutralPosition;
        plantarTorque = 0;
        plantarMode = maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode;
    }

    return;
}

void pathPlannerDorsiflexion(Reading reading){
    auto time = high_resolution_clock::now();
    auto currentTimeGap = duration_cast<microseconds>(time-timeIC);
    auto eventTimeGap = duration_cast<microseconds>(timeOFO - timeIC);
    auto footOffTimeGap = duration_cast<microseconds>(timeFO - timeIC);

    double currentCyclePercentage = currentTimegap / eventTimeGap * 0.12;
    double footOffPercentage = footOffTimeGap / eventTimeGap * 0.12;

    // After Initial Contact, deactivate dorsiflexion.
    if (currentCyclePercentage < downTimeDF){
        dorsiPosition = dorsiNeutralPosition + (downTimeDF - currentCyclePercentage);
        dorsiTorque = 0;
        dorsiMode = maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode;
    }
    // Zero torque control mode until foot off
    else if (footOffPercentage < 0) {
        dorsiPosition = dorsiNeutralPosition;
        dorsiTorque = 0;
        dorsiMode = maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode;
    }
    // Activate dorsiflexion
    else if (currentCyclePercentage < footOffPercentage + upTimeDF){
        dorsiPosition = dorsiNeutralPosition;
        dorsiTorque = 0;
        dorsiMode = maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode;
    }
    // Hold dorsiflexion
    else{
        dorsiPosition = dorsiNeutralPosition;
        dorsiTorque = 0;
        dorsiMode = maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode;
    }


    return;
}

void callbackGaitPhase(const std_msgs::int::ConstPtr& msg){
    if (msg->data == 0){     
        timeIC = high_resolution_clock::now();
    }
    else if (msg->data == 1){
        timeOFO = high_resolution_clock::now();
    }
    else if (msg->data == 2){
        timeFO = high_resolution_clock::now();
    }   
    else {
        std::cout << "Wrong Gait Phase detected" << std::endl;
    }
    return;
}

void worker()
{
    dorsiNeutralPosition = 0;
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
                    pathPlanner(reading);
                    maxon::Command command;
                    command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
                    command.setTargetPosition(plantarNeutralPosition + dirPlantar * plantarPosition);
                    command.setTargetTorque(dirPlantar * plantarTorque);
                    maxon_slave_ptr->stageCommand(command);

                }
                else if (slave->getName() == "Dorsi"){
                    command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode);
                    command.setTargetPosition(dorsiPosition + dorsiNeutralPosition);
                    maxon_slave_ptr->stageCommand(command);
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
    String configPath;
    n.getParam("/rr", rr);
    n.getParam("/afo_controller/configPath", configPath);
    ros::Rate loop_rate(rr);
    ros::Subscriber afo_gaitPhase = n.subscribe<std_msgs::Int>("/afo_predictor/gaitEvent", 1, callbackGaitPhase);

    std::signal(SIGINT, signal_handler);

    configurator = std::make_shared<EthercatDeviceConfigurator>(configPath);

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