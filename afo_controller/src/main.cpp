#include "../include/main.hpp"

double cubic(double init_time, double final_time, double current_time){
    if (current_time > final_time) return 1;
    if (current_time < init_time) return 0;

    double duration = final_time - init_time;
    double t = (current_time - init_time) / duration;
    return 3 * pow(t,2) - 2 * pow(t,3);
}
double pathPlannerPF_MH(){
    auto time = high_resolution_clock::now();
    duration<double, micro> currentTimeGap = time - timeCuePF;
    
    double currentTime = currentTimeGap.count() / 10;
    plantarPosition = 0;
    plantarMode = maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode;
    if (currentTime < 15){
        plantarTorque = currentTime / 15.0;
    }
    else if (currentTime < 45){
        plantarTorque = 1;
    }
    else if (currentTime < 60){
        plantarTorque = (60 - currentTime) / 15.0;
    }

    return 0.1;
}

double pathPlannerDF_MH(){
    auto time = high_resolution_clock::now();
    duration<double, micro> currentTimeGap = time - timeCueDF;
    
    double currentTime = currentTimeGap.count() / 10;
    dorsiPosition = 0;
    dorsiMode = maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode;
    if (currentTime < 15){
        dorsiTorque = currentTime / 15.0;
    }
    else if (currentTime < 45){
        dorsiTorque = 1;
    }
    else if (currentTime < 60){
        dorsiTorque = (60 - currentTime) / 15.0;
    }

    return 0.1;
}

double pathPlannerPlantarflexion(){
     auto time = high_resolution_clock::now();

     if (forced_trigger){
        duration<double, micro> forcedTriggerTimeGap = time - timeFT;
        if (forcedTriggerTimeGap.count() > cycleTime) {
            timeFT = time;
             timeIC = timeFT;
         }
         if (forcedTriggerTimeGap.count() > cycleTime * stance_time){
             if (timeFO < timeIC) timeFO = time;
         }
     }
    double currentCyclePercentage;
    duration<double, micro> currentTimeGap = time - timeIC;
    duration<double, micro> eventGap;
    
    currentCyclePercentage = (currentTimeGap.count()) / cycleTime;    // 
    eventGap = timeFO - timeIC;
    // Dummy variable to simplify formulation.
    double t;   

    // Relax after Foot-off
    if (eventGap.count() > 0){
        plantarPosition = 0;
        duration<double, micro> time_span = time - timeFO;
        plantarTorque = plantarStopTorque * (1 - cubic(0, relaxTime, time_span.count()/cycleTime));
        plantarMode = maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode;
    }
    // Before actuation
    else if (currentCyclePercentage < startTimePF){
        plantarPosition = 0;
        plantarTorque = 0;
        plantarMode = maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode;
    }
    // Start plantarflexion, torque increasing
    else if (currentCyclePercentage < startTimePF + riseTimePF){
        t = currentCyclePercentage - startTimePF;
        
        plantarPosition = 0;
        plantarTorque = cubic(0, riseTimePF, t);
        plantarMode = maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode;     
    }
    // Still Plantarflexion, torque decreasing
    else if (currentCyclePercentage < startTimePF + riseTimePF + flatTimePF){
        plantarTorque = 1;
        plantarMode = maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode; 
    }
    else if (currentCyclePercentage < endTimePF){
        t = currentCyclePercentage - startTimePF - riseTimePF - flatTimePF;

        plantarPosition = 0;
        plantarTorque = 1 - cubic(0, fallTimePF, t);
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

     if (forced_trigger){
         duration<double, micro> forcedTriggerTimeGap = time - timeFT;
         if (forcedTriggerTimeGap.count() > cycleTime) {
             timeFT = time;
             timeIC = timeFT;
         }
         if (forcedTriggerTimeGap.count() > cycleTime * stance_time){
             if (timeFO < timeIC) timeFO = time;
         }
     }
    double currentCyclePercentage, footOffPercentage;
    duration<double, micro> currentTimeGap = time - timeIC;
    
    currentCyclePercentage = (currentTimeGap.count()) / cycleTime;    // unit conversion: trigger_layback ms to us
    duration<double, micro> footOffTimeGap = timeFO - timeIC;
    footOffPercentage = (footOffTimeGap.count() + trigger_layback_ms * 1000.0) / cycleTime;

    if(footOffPercentage < 0){
        dorsiPosition = 0;
        dorsiTorque = dorsiStopTorque * (1 - cubic(0, relaxTime / cycleTime, currentCyclePercentage));
        dorsiMode = maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode;
    }
    else if (currentCyclePercentage < footOffPercentage + riseTimeDF){
        dorsiPosition = 0;
        dorsiTorque = cubic(footOffPercentage, footOffPercentage + riseTimeDF, currentCyclePercentage);
        dorsiMode = maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode;
    }
    else{
        dorsiPosition = 0;
        dorsiTorque = 1;
        dorsiMode = maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode;
    }
    

    dorsiTorque = min(max(dorsiTorque, 0.0), 1.0);
    dorsiPosition = min(max(dorsiPosition, 0.0), 1.0);
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

void callbackGaitPhaseAffected(const std_msgs::Int16ConstPtr& msg){
    if (msg->data == 1){
        timeIC = high_resolution_clock::now();
        dorsiStopTorque = dorsiCurrentTorque / maxTorqueDorsi;
    }
    else if (msg->data == 2){ 
        timeFO = high_resolution_clock::now();
        plantarStopTorque = plantarTorque;
    }
    else
        std::cout << "Wrong Gait Phase Detected - Affected Side" << std::endl;

    setGaitEventAffected = true;

    return;
}

void callbackGaitPhaseNonAffected(const std_msgs::Int16ConstPtr& msg){
    if (msg->data == 2){
        timeOFO = high_resolution_clock::now();
        eventTimeGap = timeOFO - timeIC;
}
    else if (msg->data != 1){
        std::cout << "Wrong Gait Phase Detected - Non Affected Side" << std::endl;        
    }

    setGaitEventNonAffected = true;

    return;
}

void callbackForcedTrigger(const std_msgs::BoolConstPtr& msg){
    trigger_layback_ms = msg->data * 1000.0;
    if (!forced_trigger) timeFT = high_resolution_clock::now();
    forced_trigger = true;
    setGaitEventAffected = true;
    setGaitEventNonAffected = true;
    return;
}

void callbackShutdown(const std_msgs::BoolConstPtr& msg){
    if(ros::isStarted()){
        ros::shutdown();
        ros::waitForShutdown();
    }
}

void callbackMaxTorque(const std_msgs::Float32MultiArray::ConstPtr& msg){
    maxTorquePlantar = msg->data[0];
    maxTorqueDorsi = msg->data[1];
    std_msgs::Float32 m_p, m_d;
    m_p.data = maxTorquePlantar;
    m_d.data = maxTorqueDorsi;
    afo_configuration_maxTorquePlantar.publish(m_p);
    afo_configuration_maxTorqueDorsi.publish(m_d);
}

void callbackRiseTime(const std_msgs::Float32MultiArray::ConstPtr& msg){
    riseTimePF = msg->data[0];
    riseTimeDF = msg->data[1];
    std_msgs::Float32 m_p, m_d;
    m_p.data = riseTimePF;
    m_d.data = riseTimeDF;
    afo_configuration_riseTimePlantar.publish(m_p);
    afo_configuration_riseTimeDorsi.publish(m_d);
}

void callbackFallTime(const std_msgs::Float32MultiArray::ConstPtr& msg){
    fallTimePF = msg->data[0];
    fallTimeDF = msg->data[1];
    std_msgs::Float32 m_p, m_d;
    m_p.data = fallTimePF;
    m_d.data = fallTimeDF;
    afo_configuration_fallTimePlantar.publish(m_p);
    afo_configuration_fallTimeDorsi.publish(m_d);
}

void callbacktriggerTime(const std_msgs::Float32MultiArray::ConstPtr& msg){
    startTimePF = msg->data[0];
    startTimeDF = msg->data[1];
    std_msgs::Float32 m_p, m_d;
    m_p.data = startTimePF;
    m_d.data = startTimeDF;
    afo_configuration_startTimePlantar.publish(m_p);
    afo_configuration_startTimeDorsi.publish(m_d);
}

void callbackCycleTime(const std_msgs::Float32ConstPtr& msg){
    cycleTime = msg->data * 1000000.0;
    std_msgs::Float32 m;
    m.data = msg->data;
    afo_configuration_cycle_time.publish(m);

}

void callbackPlantarTriggerTime(const std_msgs::Float32ConstPtr& msg){
	startTimePF = msg->data;
	std_msgs::Float32 m;
m.data = msg->data;
afo_configuration_startTimePlantar.publish(m);
}

void callbackPlantarRun(const std_msgs::BoolConstPtr& msg){
    plantarRun = msg->data;
}

void callbackDorsiRun(const std_msgs::BoolConstPtr& msg){
    dorsiRun = msg->data;
    std_msgs::Float32 m;
    m.data = dorsiNeutralPosition;
    afo_configuration_dorsiNeutralPosition.publish(m);
}

void callbackMHPF_run(const std_msgs::BoolConstPtr& msg){
    timeCuePF = high_resolution_clock::now();
    setPF_cue_MH = true;
}

void callbackMHDF_run(const std_msgs::BoolConstPtr& msg){
    timeCueDF = high_resolution_clock::now();
    setDF_cue_MH = true;
}

void worker()
{
    // Define Output file stream for controller logging.
    std::time_t t = std::time(0);
	std::tm* now = std::localtime(&t);
	string now_str = to_string(now->tm_mday) + "_" + to_string(now->tm_hour) + "_" + to_string(now->tm_min);
    
    std_msgs::Float32 m;
    
    m.data = maxTorquePlantar;
    afo_configuration_maxTorquePlantar.publish(m);

    m.data = maxTorqueDorsi;
    afo_configuration_maxTorqueDorsi.publish(m);

    m.data = riseTimePF;
    afo_configuration_riseTimePlantar.publish(m);

    m.data = riseTimeDF;
    afo_configuration_riseTimeDorsi.publish(m);

    m.data = fallTimePF;
    afo_configuration_fallTimePlantar.publish(m);

    m.data = fallTimeDF;
    afo_configuration_fallTimeDorsi.publish(m);

    m.data = startTimePF;
    afo_configuration_startTimePlantar.publish(m);

    m.data = startTimeDF;
    afo_configuration_startTimeDorsi.publish(m);

    m.data = dirPlantar;
    afo_configuration_dirPlantar.publish(m);

    m.data = cycleTime;
    afo_configuration_cycle_time.publish(m);

    m.data = maxPositionDorsi;
    afo_configuration_maxPositionDorsi.publish(m);

    m.data = dorsiPreTension;
    afo_configuration_dorsiPreTension.publish(m);

    m.data = plantarPreTension;
    afo_configuration_plantarPreTension.publish(m);

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
			                cout << "Dorsi Zeroing Done: " << reading.getActualTorque() << endl;
                            std_msgs::Bool m_dz;
                            m_dz.data = true;
                            afo_dorsi_zeroing_done.publish(m_dz);
                            m.data = dorsiNeutralPosition;
                            afo_configuration_dorsiNeutralPosition.publish(m);
                            break;
                        }
                        command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
                        command.setTargetPosition(reading.getActualPosition() + dorsiZeroingIncrement * dirDorsi);
                        command.setTargetTorque(dorsiPreTension * dirDorsi);
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
        else{
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
                        if (setPF_cue_MH){
                            pathPlannerPF_MH();
                        }
                        if(plantarRun){
                            command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
                            command.setTargetPosition(plantarNeutralPosition + plantarPosition * dirPlantar);
                            command.setTargetTorque(dirPlantar * (plantarPreTension + maxTorquePlantar * plantarTorque));
                        }
                        else{
                            command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
                            command.setTargetPosition(plantarNeutralPosition + 0 * dirPlantar);
                            command.setTargetTorque(dirPlantar * (plantarPreTension + maxTorquePlantar * 0));
                        }
                        maxon_slave_ptr->stageCommand(command);
                        
                        float plantarModeInt;
                        if (plantarMode == maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode)
                            plantarModeInt = 1.0;
                        else
                            plantarModeInt = 2.0;
                        plantarCurrentTorque = reading.getActualTorque();
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
                    }
                    else if (slave->getName() == "Dorsi"){
                        if (setGaitEventNonAffected && setGaitEventAffected){
                            currentTimePercentage = pathPlannerDorsiflexion(reading);
                        }
                        if (setDF_cue_MH){
                            pathPlannerDF_MH();
                        }
                        if (dorsiRun){
                            dorsiInputMode = dorsiMode;
                            dorsiPositionInput = dorsiNeutralPosition + maxPositionDorsi * dorsiPosition * dirDorsi;
                            dorsiTorqueInput = dirDorsi * (maxTorqueDorsi * dorsiTorque + dorsiPreTension);
                        }
                        else{
                            dorsiInputMode = maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode;
                            dorsiPositionInput = dorsiNeutralPosition + maxPositionDorsi * 0 * dirDorsi;
                            dorsiTorqueInput = dirDorsi * (maxTorqueDorsi * 0 + dorsiPreTension);
                            dorsiNeutralPosition = reading.getActualPosition();
                        }
                            
                        command.setModeOfOperation(dorsiInputMode);
                        command.setTargetTorque(dorsiTorqueInput);
                        command.setTargetPosition(dorsiPositionInput);
                        maxon_slave_ptr->stageCommand(command);
                         
                        float dorsiModeInt, dorsiInputModeInt;
                        if (dorsiMode == maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode)
                            dorsiModeInt = 1.0;
                        else
                            dorsiModeInt = 2.0;

                        if (dorsiInputMode == maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode)
                            dorsiInputModeInt = 1.0;
                        else
                            dorsiInputModeInt = 2.0;
                        dorsiCurrentTorque = reading.getActualTorque();
                        msg_motor_dorsi.data.clear();
                        msg_motor_dorsi.data.push_back(currentTimePercentage);
                        msg_motor_dorsi.data.push_back(dorsiModeInt);
                        msg_motor_dorsi.data.push_back(dirDorsi * (maxTorqueDorsi * dorsiTorque + dorsiPreTension));
                        msg_motor_dorsi.data.push_back(dorsiNeutralPosition + maxPositionDorsi * dorsiPosition * dirDorsi);
                        msg_motor_dorsi.data.push_back(reading.getActualCurrent());
                        msg_motor_dorsi.data.push_back(reading.getActualTorque());
                        msg_motor_dorsi.data.push_back(reading.getActualPosition());
                        msg_motor_dorsi.data.push_back(reading.getActualVelocity());
                        msg_motor_dorsi.data.push_back(reading.getBusVoltage());
                        msg_motor_dorsi.data.push_back(dorsiInputModeInt);
                        msg_motor_dorsi.data.push_back(dorsiStage);
                        
                        afo_motor_data_dorsi.publish(msg_motor_dorsi);
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
    afo_gait_nonparetic = n.subscribe("/afo_detector/gait_nonparetic", 1, callbackGaitPhaseNonAffected);
    afo_gait_paretic = n.subscribe("/afo_detector/gait_paretic", 1, callbackGaitPhaseAffected);
    afo_gui_max_torque = n.subscribe("/afo_gui/max_torque", 1, callbackMaxTorque);
    afo_gui_cycle_time = n.subscribe("/afo_gui/cycle_time", 1, callbackCycleTime);
    afo_gui_plantar_run = n.subscribe("/afo_gui/plantar_run", 1, callbackPlantarRun);
ros::Subscriber afo_gui_plantar_trigger_time = n.subscribe("/afo_gui/plantar_trigger_time", 1, callbackPlantarTriggerTime);
    afo_gui_dorsi_run = n.subscribe("/afo_gui/dorsi_run", 1, callbackDorsiRun);
    afo_gui_mh_pf_run = n.subscribe("/afo_gui/run_pf_mh", 1, callbackMHPF_run);
    afo_gui_mh_df_run = n.subscribe("/afo_gui/run_df_mh", 1, callbackMHDF_run);
    afo_gui_forced_trigger = n.subscribe("/afo_gui/forced_trigger", 1, callbackForcedTrigger);

    afo_motor_data_plantar = n.advertise<std_msgs::Float32MultiArray>("/afo_controller/motor_data_plantar", 10);
    afo_motor_data_dorsi = n.advertise<std_msgs::Float32MultiArray>("/afo_controller/motor_data_dorsi", 10);
    afo_configuration_cycle_time = n.advertise<std_msgs::Float32>("/afo_controller/cycle_time", 10);
    afo_configuration_maxTorquePlantar = n.advertise<std_msgs::Float32>("/afo_controller/maxTorquePlantar", 10);
    afo_configuration_maxTorqueDorsi = n.advertise<std_msgs::Float32>("/afo_controller/maxTorqueDorsi", 10);
    afo_configuration_riseTimePlantar = n.advertise<std_msgs::Float32>("/afo_controller/rise_time_plantar", 10);
    afo_configuration_riseTimeDorsi = n.advertise<std_msgs::Float32>("/afo_controller/rise_time_dorsi", 10);
    afo_configuration_fallTimePlantar = n.advertise<std_msgs::Float32>("/afo_controller/fall_time_plantar", 10);
    afo_configuration_fallTimeDorsi = n.advertise<std_msgs::Float32>("/afo_controller/fall_time_dorsi", 10);
    afo_configuration_startTimePlantar = n.advertise<std_msgs::Float32>("/afo_controller/start_time_plantar", 10);
    afo_configuration_startTimeDorsi = n.advertise<std_msgs::Float32>("/afo_controller/start_time_dorsi", 10);
    afo_configuration_maxPositionDorsi = n.advertise<std_msgs::Float32>("/afo_controller/maxPositionDorsi", 10);
    afo_configuration_dorsiPreTension = n.advertise<std_msgs::Float32>("/afo_controller/dorsi_pretension", 10);
    afo_configuration_plantarPreTension = n.advertise<std_msgs::Float32>("/afo_controller/plantar_pretension", 10);
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

    plantarRun = false;
    dorsiRun = false;
    
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
