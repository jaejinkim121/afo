#include "../include/main.hpp"

void worker()
{
    // Define Output file stream for controller logging.
    std::time_t t = std::time(0);
	std::tm* now = std::localtime(&t);
	string now_str = to_string(now->tm_mday) + "_" + to_string(now->tm_hour) + "_" + to_string(now->tm_min);
    ofstream outFileController;
    outFileController.open("/home/dl/catkin_ws/src/afo/afo_controller/log/controller_" + now_str + ".csv");
    outFileController << 
        "Max Plantar Torque=" << maxTorquePlantar << 
        ", Max Dorsi Torque=" << maxTorqueDorsi << 
        ", start time=" << startTime << 
        ", endTime=" << endTime <<
        ", upTimeRatio=" << upTimeRatio <<
        ", dirPlantar=" << dirPlantar <<
        ", ";

    // Initialize master
    bool rtSuccess = true;
    for(const auto & master: configurator->getMasters())
    {
        rtSuccess &= master->setRealtimePriority(99);
    }
    bool maxonEnabledAfterStartup = false;
    maxon::ModeOfOperationEnum dorsiInputMode;
    double dorsiPositionInput, dorsiTorqueInput;
    /*
    ** The communication update loop.
    ** This loop is supposed to be executed at a constant rate.
    ** The EthercatMaster::update function incorporates a mechanism
    ** to create a constant rate.
     */
    auto next = steady_clock::now();
    while(!abrt)
    {
        if(!motorInit){
            continue;
        }
        // ------------------------------- Dorsiflexion zeroing process start -------------------------------------------- //
        if (!isDorsiZeroing){
            for(const auto & master: configurator->getMasters() ){
                master->update(ecat_master::UpdateMode::StandaloneEnforceRate); // TODO fix the rate compensation (Elmo reliability problem)!!
            }               
            for(const auto & slave:configurator->getSlaves())
            {  
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
                    maxon::Command command;

                    if (slave->getName() == "Plantar"){
                        command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
                        command.setTargetPosition(0);
                        command.setTargetTorque(plantarPreTension * dirPlantar);
                        maxon_slave_ptr->stageCommand(command);
                    }
                    else if (slave->getName() == "Dorsi"){
                        auto reading = maxon_slave_ptr->getReading();
                        if (reading.getActualTorque() * dirDorsi > dorsiPreTension){
				            if (dorsiBufferFlushingIndex++ < 15){
                                continue;
                            }
                            isDorsiZeroing = true;
                            dorsiNeutralPosition = reading.getActualPosition();
			                outFileController << "dorsiNeutralPosition=" << dorsiNeutralPosition << endl;
		                    cout << "Dorsi Zeroing Done: " << reading.getActualTorque() << endl;
                            std_msgs::Bool m;
                            m.data = true;
                            afo_dorsi_zeroing_done.publish(m);
                            break;
                        }
                        command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode);
                        command.setTargetPosition(reading.getActualPosition() + dorsiZeroingIncrement * dirDorsi);
                        command.setTargetTorque(0);
                        maxon_slave_ptr->stageCommand(command);
                    }
                }
                else
                {
                    MELO_WARN_STREAM("Maxon '" << maxon_slave_ptr->getName()
                                                                            << "': " << maxon_slave_ptr->getReading().getDriveState());
                }
            }
            maxonEnabledAfterStartup = true;
        }
        /* -----------------------------    Dorsiflexion Zeroing Process Done     ---------------------------------*/
        /*
        ** Update each master.
        ** This sends tha last staged commands and reads the latest readings over EtherCAT.
        ** The StandaloneEnforceRate update mode is used.
        ** This means that average update rate will be close to the target rate (if possible).
         */
        // CONTROL MAIN LOOP
        else if (!motorRun){
            for(const auto & slave:configurator->getSlaves()){
                std::shared_ptr<maxon::Maxon> maxon_slave_ptr = std::dynamic_pointer_cast<maxon::Maxon>(slave);

                maxon::Command command;
                if (slave->getName() == "Plantar"){
                    command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
                    command.setTargetPosition(plantarNeutralPosition + 0 * dirPlantar);
                    command.setTargetTorque(dirPlantar * (plantarPreTension + maxTorquePlantar * 0));
                    maxon_slave_ptr->stageCommand(command);
                }
                else{
                    command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
                    command.setTargetTorque(dirDorsi * (maxTorqueDorsi * 0 + dorsiPreTension));
                    command.setTargetPosition(dorsiNeutralPosition);
                    maxon_slave_ptr->stageCommand(command);
                }
            }
        }
        
        else{

            if (!motorRun){
                
            }

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
                    maxon::Command command;
                    auto reading = maxon_slave_ptr->getReading();
                    double currentTimePercentage;
                    // CONTROL LOOP MAIN BODY
                    if (slave->getName() == "Plantar"){
                        if (setGaitEventNonAffected && setGaitEventAffected){
                            currentTimePercentage = pathPlannerPlantarflexion();
                        }
                        command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
                        command.setTargetPosition(plantarNeutralPosition + plantarPosition * dirPlantar);
                        command.setTargetTorque(dirPlantar * (plantarPreTension + maxTorquePlantar * plantarTorque));
                        maxon_slave_ptr->stageCommand(command);
                        
                        float plantarModeInt;
                        if (plantarMode == maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode)
                            plantarModeInt = 1.0;
                        else
                            plantarModeInt = 2.0;

                        msg_motor_plantar.data.clear();
                        msg_motor_plantar.data.push_back(currentTimePercentage);
                        msg_motor_plantar.data.push_back(plantarModeInt);
                        msg_motor_plantar.data.push_back(dirPlantar * (plantarPreTension + maxTorquePlantar * plantarTorque));
                        msg_motor_plantar.data.push_back(plantarPosition);
                        msg_motor_plantar.data.push_back(reading.getActualCurrent());
                        msg_motor_plantar.data.push_back(reading.getActualTorque());
                        msg_motor_plantar.data.push_back(reading.getActualPosition());
                        msg_motor_plantar.data.push_back(reading.getActualVelocity());
                        msg_motor_plantar.data.push_back(reading.getBusVoltage());
                        
                        afo_motor_data_plantar.publish(msg_motor_plantar);
                        // outFileController << "plantar, " << ros::Time::now() << ", " << currentTimePercentage << ", "
                        //     << plantarMode << ", " 
                        //     << dirPlantar * (plantarPreTension + maxTorquePlantar * plantarTorque) << ", " 
                        //     << plantarNeutralPosition + plantarPosition * dirPlantar << ", " 
                        //     << reading.getActualCurrent() << ", " 
                        //     << reading.getActualTorque() << ", " 
                        //     << reading.getActualPosition() << ", " 
                        //     << reading.getActualVelocity() << ", " 
                        //     << reading.getBusVoltage() << endl;

                    }
                    else if (slave->getName() == "Dorsi"){
                        if (setGaitEventNonAffected && setGaitEventAffected){
                            currentTimePercentage = pathPlannerDorsiflexion(reading);
                        }
                            
                        dorsiInputMode = dorsiMode;
                        dorsiPositionInput = dorsiNeutralPosition + maxPositionDorsi * dorsiPosition * dirDorsi;
                        dorsiTorqueInput = dirDorsi * (maxTorqueDorsi * dorsiTorque + dorsiPreTension);
                            
                        command.setModeOfOperation(dorsiInputMode);
                        command.setTargetTorque(dorsiTorqueInput);
                        command.setTargetPosition(dorsiPositionInput);
                        maxon_slave_ptr->stageCommand(command);
                         
                        float dorsiModeInt;
                        if (dorsiMode == maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode)
                            dorsiModeInt = 1.0;
                        else
                            dorsiModeInt = 2.0;

                        msg_motor_dorsi.data.clear();
                        msg_motor_dorsi.data.push_back(currentTimePercentage);
                        msg_motor_dorsi.data.push_back(dorsiModeInt);
                        msg_motor_dorsi.data.push_back(dorsiTorqueInput);
                        msg_motor_dorsi.data.push_back(dorsiPositionInput);
                        msg_motor_dorsi.data.push_back(reading.getActualCurrent());
                        msg_motor_dorsi.data.push_back(reading.getActualTorque());
                        msg_motor_dorsi.data.push_back(reading.getActualPosition());
                        msg_motor_dorsi.data.push_back(reading.getActualVelocity());
                        msg_motor_dorsi.data.push_back(reading.getBusVoltage());
                        
                        afo_motor_data_dorsi.publish(msg_motor_dorsi);
                            
                        // outFileController << "dorsi, " 
                        //     << ros::Time::now() << ", " 
                        //     << currentTimePercentage << ", "
                        //     << dorsiInputMode << ", " 
                        //     << dorsiTorqueInput << ", " 
                        //     << dorsiPositionInput << ", " 
                        //     << reading.getActualCurrent() << ", " 
                        //     << reading.getActualTorque() << ", " 
                        //     << reading.getActualPosition() << ", " 
                        //     << reading.getActualVelocity() << ", " 
                        //     << reading.getBusVoltage() << endl;
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
        next += microseconds(etherCatCommunicationRate);
        std::this_thread::sleep_until(next);
    }							
}

double cubic(double init_time, double final_time, double current_time){
    double duration = final_time - init_time;
    double t = (current_time - init_time) / duration;
    return 2 * pow(t,3) - 3 * pow(t,2);
}

double pathPlannerPlantarflexion(){
    auto time = high_resolution_clock::now();
    duration<double, micro> currentTimeGap = time - timeIC;
    double currentCyclePercentage;
    if (controlMode == EST)
        currentCyclePercentage = currentTimeGap.count() / eventTimeGap.count() * 0.12;
    else
        currentCyclePercentage = currentTimeGap.count() / periodPreset;

    // Dummy variable to simplify formulation.
    double t;   

    // Before actuation
    if (currentCyclePercentage < startTime){
        plantarPosition = 0;
        plantarTorque = 0;
        plantarMode = maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode;
    }
    // Start plantarflexion, torque increasing
    else if (currentCyclePercentage < startTime + onTime * upTimeRatio){
        t = currentCyclePercentage - startTime;
        
        plantarPosition = 0;
        // Acceleration
        if (t < 0.5 * upTimeRatio * onTime){
            plantarTorque = 0.5 * acc * pow(t,2);
        }
        // Deceleration
        else{
            plantarTorque = 1 - 0.5 * acc * pow(t - onTime * upTimeRatio, 2);
        }
        plantarMode = maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode;     
    }
    // Still Plantarflexion, torque decreasing
    else if (currentCyclePercentage < endTime){
        t = currentCyclePercentage - startTime - onTime * upTimeRatio;

        plantarPosition = 0;

        // Acceleration
        if (t < 0.5 * (1 - upTimeRatio) * onTime) {
            plantarTorque = 1 - 0.5 * acc * upTimeRatio / (1 - upTimeRatio) * pow(t, 2);
        }
        // Deceleration
        else {
            plantarTorque = 0.5 * acc * upTimeRatio / (1 - upTimeRatio) * pow(t - onTime * (1 - upTimeRatio), 2);
        }
        plantarMode = maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode;
    }
    // After end of plantarflexion
    else {
        plantarPosition = 0;
        plantarTorque = 0;
        plantarMode = maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode;
    }
    plantarTorque = min(max(plantarTorque, 0.0), 1.0);
    return currentCyclePercentage;
}

double pathPlannerDorsiflexion(maxon::Reading reading){
    auto time = high_resolution_clock::now();
    duration<double, micro> currentTimeGap = time - timeIC;
    duration<double, micro> footOffTimeGap = timeFO - timeIC;
    double currentCyclePercentage, footOffPercentage;
    if (controlMode == EST){
        currentCyclePercentage = currentTimeGap.count() / eventTimeGap.count() * 0.12;
        footOffPercentage = footOffTimeGap.count() / eventTimeGap.count() * 0.12;
    }
    else{
        currentCyclePercentage = currentTimeGap.count() / periodPreset;
        footOffPercentage = footOffTimeGap.count() / periodPreset;
    }

    // After Initial Contact, deactivate dorsiflexion.
    // stage 1
    if (currentCyclePercentage < downtimeDF){
        dorsiStage = 1;
        dorsiPosition = 1 - currentCyclePercentage / downtimeDF;
        dorsiTorque = 0;
        dorsiMode = maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode;
    }
    // Pretension torque control mode until foot off
    // stage 2
    else if (footOffPercentage < 0) {
        dorsiStage = 2;
        dorsiPosition = 0;
        dorsiTorque = 0;
        dorsiMode = maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode;
    }
    // Sustain target position
    // stage 3
    else if (
        dorsiStage == 3 || 
        abs(reading.getActualPosition() - dorsiNeutralPosition - maxPositionDorsi) < positionDiffLimit)
        {
        dorsiStage = 3;
        dorsiPosition = 1;
        dorsiTorque = 0;
        dorsiMode = maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode;
    }
    // make increment to achieve target position
    // stage 4
    else if (currentCyclePercentage < footOffPercentage + uptimeDF){
        if (dorsiStage !=4){
            dorsiStage = 4;
            dorsiTorqueDir = reading.getActualPosition() - dorsiNeutralPosition > maxPositionDorsi;
        }

        if (dorsiTorqueDir) dorsiTorque = (reading.getActualTorque() - dorsiTorqueSlope) / maxTorqueDorsi;
        else dorsiTorque = (reading.getActualTorque() + dorsiTorqueSlope) / maxTorqueDorsi;

        dorsiPosition = 0;
        dorsiMode = maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode;
    }

    dorsiTorque = min(max(dorsiTorque, 0.0), 1.0);
    return currentCyclePercentage;
}

void callbackGaitPhase(const std_msgs::Int16MultiArray::ConstPtr& msg){
    int nonAffected, affected;
    nonAffected = msg->data[0];   // non-affected side
    affected = msg->data[1];   // affected side

    if (nonAffected == IC){
        //
    }
    else if (nonAffected == FO){
        timeOFO = high_resolution_clock::now();
        eventTimeGap = timeOFO - timeIC;
        setGaitEventNonAffected = true;
    }

    if (affected == IC){
        timeIC = high_resolution_clock::now();
        setGaitEventAffected = true;
    }
    else if (affected == FO){
        timeFO = high_resolution_clock::now();
        setGaitEventAffected = true;
    }
    
}

void callbackGaitPhaseAffected(const std_msgs::Int16::ConstPtr& msg){
    if (msg->data == 0) 
        timeIC = high_resolution_clock::now();
    else if (msg->data == 1) 
        timeFO = high_resolution_clock::now();
    else
        std::cout << "Wrong Gait Phase Detected - Affected Side" << std::endl;

    setGaitEventAffected = true;

    return;
}

void callbackGaitPhaseNonAffected(const std_msgs::Int16::ConstPtr& msg){
    if (msg->data == 1){
        timeOFO = high_resolution_clock::now();
        eventTimeGap = timeOFO - timeIC;
}
    else
        std::cout << "Wrong Gait Phase Detected - Non Affected Side" << std::endl;

    setGaitEventNonAffected = true;

    return;
}

void callbackShutdown(const std_msgs::BoolConstPtr& msg){
    if(ros::isStarted()){
        ros::shutdown();
        ros::waitForShutdown();
    }
}

void callbackMaxTorque(const std_msgs::Float32ConstPtr& msg){
    maxTorquePlantar = msg->data;
    maxTorqueDorsi = msg->data;
    std::cout << "maximum torque set to " << maxTorquePlantar << std::endl;
}

void callbackCycleTime(const std_msgs::Float32ConstPtr& msg){
    cycleTime = msg->data;
    std::cout << "cycle time set to " << cycleTime << std::endl;
}

void callbackMotorRun(const std_msgs::BoolConstPtr& msg){
    motorRun = true;
    motorInit = true;
    std::cout << "Motor Start" << std::endl;
}

void callbackMotorStop(const std_msgs::BoolConstPtr& msg){
    motorRun = false;
    std::cout << "Motor Stop" << std::endl;
}
/*
** Handle the interrupt signal.
** This is the shutdown routine.
** Note: This logic is executed in a thread separated from the communication update!
 */
void terminateMotor(int sig)
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
    std::cout << "Motor Controller Shutdown" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    exit(0);
}


/*
** Program entry.
** Pass the path to the setup.yaml file as first command line argument.
 */
int main(int argc, char**argv)
{    
    // Define ROS
    ros::init(argc, argv, "afo_controller");
    ros::NodeHandle n;
    int rr;
    string configPath;
    n.getParam("/rr", rr);
    n.getParam("/afo_controller/configPath", configPath);
    ros::Rate loop_rate(rr);

    // ros::Subscriber afo_gaitPhaseAffected = n.subscribe("/afo_predictor/gaitEventAffected", 1, callbackGaitPhaseAffected);
    // ros::Subscriber afo_gaitPhaseNonAffected = n.subscribe("/afo_predictor/gaitEventNonAffected", 1, callbackGaitPhaseNonAffected);

    afo_shutdown_sub = n.subscribe("/afo_sync/shutdown", 1, callbackShutdown);
    afo_gaitPhase = n.subscribe("/afo_detector/gaitPhase", 1, callbackGaitPhase);
    afo_gui_max_torque = n.subscribe("/afo_gui/max_torque", 1, callbackMaxTorque);
    afo_gui_cycle_time = n.subscribe("/afo_gui/cycle_time", 1, callbackCycleTime);
    afo_gui_motor_run = n.subscribe("/afo_gui/motor_run", 1, callbackMotorRun);
    afo_gui_motor_stop = n.subscribe("/afo_gui/motor_stop", 1, callbackMotorStop);

    afo_motor_data_plantar = n.advertise<std_msgs::Float32MultiArray>("/afo_controller/motor_data_plantar", 10);
    afo_motor_data_dorsi = n.advertise<std_msgs::Float32MultiArray>("/afo_controller/motor_data_dorsi", 10);
    afo_configuration_maxTorquePlantar = n.advertise<std_msgs::Float32>("/afo_controller/maxTorquePlantar", 10);
    afo_configuration_maxTorqueDorsi = n.advertise<std_msgs::Float32>("/afo_controller/maxTorqueDorsi", 10);
    afo_configuration_startTime = n.advertise<std_msgs::Float32>("/afo_controller/startTime", 10);
    afo_configuration_endTime = n.advertise<std_msgs::Float32>("/afo_controller/endTime", 10);
    afo_configuration_upTimeRatio = n.advertise<std_msgs::Float32>("/afo_controller/upTimeRatio", 10);
    afo_configuration_dirPlantar = n.advertise<std_msgs::Float32>("/afo_controller/dirPlantar", 10);
    afo_configuration_dorsiNeutralPosition = n.advertise<std_msgs::Float32>("/afo_controller/dorsiNeutralPosition", 10);
    afo_dorsi_zeroing_done = n.advertise<std_msgs::Bool>("/afo_controller/dorsi_zeroing_done", 10);
    // 
    std::signal(SIGINT, terminateMotor);

    configurator = std::make_shared<EthercatDeviceConfigurator>(configPath); 

    // Command Initialize
    dorsiNeutralPosition = 0;
    plantarNeutralPosition = 0;
    dorsiPosition = 0;
    dorsiTorque = 0;
    plantarPosition = 0;
    plantarTorque = 0;
    plantarMode = maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode;
    dorsiMode = maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode;

    motorInit = false;
    motorRun = false;
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
    
    while(ros::ok()){
	    ros::spinOnce();
    }
    cout << "afo_controller Node - Terminate properly" << endl;    
}
