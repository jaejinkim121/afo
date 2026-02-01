#include "../include/main.hpp"


static inline std::string trim(const std::string& s) {
    size_t b = 0, e = s.size();
    while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

static inline std::vector<std::string> split_csv_3(const std::string& line) {
    // 아주 단순한 CSV(따옴표 없는) 전제. "1, 2, 3.0" 같은 형태 지원.
    std::vector<std::string> out;
    out.reserve(3);

    std::string token;
    std::stringstream ss(line);
    while (std::getline(ss, token, ',')) {
        out.push_back(trim(token));
    }
    return out;
}

std::vector<std::array<double, 3>> loadRandParamCsv(const std::string& csv_path) {
    std::ifstream fin(csv_path);
    if (!fin.is_open()) {
        throw std::runtime_error("Failed to open csv: " + csv_path);
    }

    std::vector<std::array<double, 3>> table;
    std::string line;
    int line_no = 0;

    while (std::getline(fin, line)) {
        ++line_no;
        line = trim(line);

        // 빈 줄 / 주석 줄 무시
        if (line.empty()) continue;
        if (!line.empty() && (line[0] == '#')) continue;

        auto cols = split_csv_3(line);
        if (cols.size() != 3) {
            std::ostringstream oss;
            oss << "CSV parse error: expected 3 columns at line " << line_no
                << " but got " << cols.size() << " | line: [" << line << "]";
            throw std::runtime_error(oss.str());
        }

        try {
            std::array<double, 3> row{
                std::stod(cols[0]),
                std::stod(cols[1]),
                std::stod(cols[2])
            };
            table.push_back(row);
        } catch (const std::exception& e) {
            std::ostringstream oss;
            oss << "CSV parse error: stod failed at line " << line_no
                << " | line: [" << line << "] | what: " << e.what();
            throw std::runtime_error(oss.str());
        }
    }

    if (table.empty()) {
        throw std::runtime_error("CSV is empty or contains no valid rows: " + csv_path);
    }
    return table;
}

double cubic(double init_time, double final_time, double current_time){
    if (current_time > final_time) return 1;
    if (current_time < init_time) return 0;

    double duration = final_time - init_time;
    double t = (current_time - init_time) / duration;
    return 3 * pow(t,2) - 2 * pow(t,3);
}

double pathPlannerPlantarParetic(){
    auto time = high_resolution_clock::now();
    double currentCyclePercentage;
    duration<double, micro> currentTimeGap = time - timeICP;
    duration<double, micro> eventGap;
    
    currentCyclePercentage = (currentTimeGap.count()) / cycleTime;    // 
    eventGap = timeFOP - timeICP;
    // Dummy variable to simplify formulation.
    double t;   

    // Relax after Foot-off
    if (eventGap.count() > 0){
        duration<double> time_span = time - timeFOP;
        pareticTorque = pareticStopTorque * (1 - cubic(0, relaxTime / cycleTime, time_span.count()));
    }
    // Before actuation
    else if (currentCyclePercentage < startTimePF){
        pareticTorque = 0;
    }
    // Start plantarflexion, torque increasing
    else if (currentCyclePercentage < startTimePF + riseTimePF){
        t = currentCyclePercentage - startTimePF;
        pareticTorque = cubic(0, riseTimePF, t);
    }
    else if (currentCyclePercentage < startTimePF + riseTimePF + flatTimePF){
        pareticTorque = 1.0;
    }
    // Still Plantarflexion, torque decreasing
    else if (currentCyclePercentage < startTimePF + riseTimePF + flatTimePF + fallTimePF){
        t = currentCyclePercentage - startTimePF - riseTimePF - flatTimePF;
        pareticTorque = 1 - cubic(0, fallTimePF, t);
    }
    // After end of plantarflexion
    else {
        pareticTorque = 0;
    }
    pareticTorque = min(max(pareticTorque, 0.0), 1.0);
    return currentCyclePercentage;

}

double pathPlannerPlantarNonParetic(){
    auto time = high_resolution_clock::now();
    double currentCyclePercentage;
    duration<double, micro> currentTimeGap = time - timeICN;
    duration<double, micro> eventGap;
    
    currentCyclePercentage = (currentTimeGap.count()) / cycleTime;    // 
    eventGap = timeFON - timeICN;
    // Dummy variable to simplify formulation.
    double t;   

    // Relax after Foot-off
    if (eventGap.count() > 0){
        duration<double> time_span = time - timeFOP;
        nonpareticTorque = nonpareticStopTorque * (1 - cubic(0, relaxTime / cycleTime, time_span.count()));
    }
    // Before actuation
    else if (currentCyclePercentage < startTimePF){
        nonpareticTorque = 0;
    }
    // Start plantarflexion, torque increasing
    else if (currentCyclePercentage < startTimePF + riseTimePF){
        t = currentCyclePercentage - startTimePF;
        nonpareticTorque = cubic(0, riseTimePF, t);
    }
    else if (currentCyclePercentage < startTimePF + riseTimePF + flatTimePF){
        nonpareticTorque = 1.0;
    }
    // Still Plantarflexion, torque decreasing
    else if (currentCyclePercentage < startTimePF + riseTimePF + flatTimePF + fallTimePF){
        t = currentCyclePercentage - startTimePF - riseTimePF - flatTimePF;
        nonpareticTorque = 1 - cubic(0, fallTimePF, t);
    }
    // After end of plantarflexion
    else {
        nonpareticTorque = 0;
    }
    nonpareticTorque = min(max(nonpareticTorque, 0.0), 1.0);
    return currentCyclePercentage;
}

double pathPlannerDorsiParetic(){
    auto time = high_resolution_clock::now();
    double currentCyclePercentage, footOffPercentage;

    duration<double, micro> currentTimeGap = time - timeICP;
    
    currentCyclePercentage = (currentTimeGap.count()) / cycleTime;    // unit conversion: trigger_layback ms to us
    duration<double, micro> footOffTimeGap = timeFOP - timeICP;
    footOffPercentage = (footOffTimeGap.count() + trigger_layback_ms * 1000.0) / cycleTime;

    if(footOffPercentage < 0){
        pareticTorque = pareticStopTorque * (1 - cubic(0, relaxTime / cycleTime, currentCyclePercentage));
    }
    else if (currentCyclePercentage < footOffPercentage + riseTimeDF){
        pareticTorque = cubic(footOffPercentage, footOffPercentage + riseTimeDF, currentCyclePercentage);
    }
    else{
        pareticTorque = 1;
    }

    pareticTorque = min(max(pareticTorque, 0.0), 1.0);

    return currentCyclePercentage;
}

double pathPlannerDorsiNonParetic(){
    auto time = high_resolution_clock::now();
    double currentCyclePercentage, footOffPercentage;

    duration<double, micro> currentTimeGap = time - timeICN;
    
    currentCyclePercentage = (currentTimeGap.count()) / cycleTime;    // unit conversion: trigger_layback ms to us
    duration<double, micro> footOffTimeGap = timeFON - timeICN;
    footOffPercentage = (footOffTimeGap.count() + trigger_layback_ms * 1000.0) / cycleTime;

    if(footOffPercentage < 0){
        nonpareticTorque = nonpareticStopTorque * (1 - cubic(0, relaxTime / cycleTime, currentCyclePercentage));
    }
    else if (currentCyclePercentage < footOffPercentage + riseTimeDF){
        nonpareticTorque = cubic(footOffPercentage, footOffPercentage + riseTimeDF, currentCyclePercentage);
    }
    else{
        nonpareticTorque = 1;
    }

    nonpareticTorque = min(max(nonpareticTorque, 0.0), 1.0);

    return currentCyclePercentage;
}

void callbackGaitPhase(const std_msgs::Int16MultiArray::ConstPtr& msg){
    int nonparetic, paretic;
    nonparetic = msg->data[0];   // non-affected side
    paretic = msg->data[1];   // affected side
    if (nonparetic == IC){
        timeICN = high_resolution_clock::now();
        setGaitEventNonAffected = true;
    }
    else if (nonparetic == FO){
        timeFON = high_resolution_clock::now();
        setGaitEventNonAffected = true;
    }

    if (paretic == IC){
        timeICP = high_resolution_clock::now();
        setGaitEventAffected = true;
    }
    else if (paretic == FO){
        timeFOP = high_resolution_clock::now();
        setGaitEventAffected = true;
    }
    
}


void callbackGaitPhaseAffected(const std_msgs::Int16ConstPtr& msg){
    if (msg->data == 1){
        timeICP = high_resolution_clock::now();
        
  }
    else if (msg->data == 2){ 
        timeFOP = high_resolution_clock::now();
        pareticStopTorque = pareticCurrentTorque / maxTorquePlantar;

    }
    else
        std::cout << "Wrong Gait Phase Detected - Affected Side" << std::endl;

    setGaitEventAffected = true;

    return;
}

void callbackGaitPhaseNonAffected(const std_msgs::Int16ConstPtr& msg){
    if (msg->data == 1){
        timeICN = high_resolution_clock::now();
        
  }
    else if (msg->data == 2){ 
        timeFON = high_resolution_clock::now();
        nonpareticStopTorque = nonpareticCurrentTorque / maxTorquePlantar;
    }
    else
        std::cout << "Wrong Gait Phase Detected - Affected Side" << std::endl;

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
}

void callbackRandRun(const std_msgs::BoolConstPtr& msg){
    isRand = true;
    timeRand = high_resolution_clock::now();
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
    /*
    ** The communication update loop.
    ** This loop is supposed to be executed at a constant rate.
    ** The EthercatMaster::update function incorporates a mechanism
    ** to create a constant rate.
     */
    auto next = steady_clock::now();
    
    while (!abrt){
        // CONTROL MAIN LOOP        
        for(const auto & master: configurator->getMasters() )
        {
            master->update(ecat_master::UpdateMode::StandaloneEnforceRate); // TODO fix the rate compensation (Elmo reliability problem)!!
        }
        for(const auto & slave:configurator->getSlaves())
        {  
            std::shared_ptr<maxon::Maxon> maxon_slave_ptr = std::dynamic_pointer_cast<maxon::Maxon>(slave);
            if (!maxonEnabledAfterStartup){
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
                if (slave->getName() == "Paretic"){
                    if (setGaitEventNonAffected && setGaitEventAffected){
                        currentTimePercentage = pathPlannerPlantarParetic();
                    }
                    if(plantarRun){
                        command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
                        command.setTargetPosition(plantarNeutralPosition + plantarPosition * dirPlantar);
                        command.setTargetTorque(dirParetic * (plantarPreTension + maxTorquePlantar * pareticTorque));
                    }
                    else{
                        command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
                        command.setTargetPosition(plantarNeutralPosition + 0 * dirPlantar);
                        command.setTargetTorque(dirParetic * plantarPreTension);
                    }
                    maxon_slave_ptr->stageCommand(command);
                    
                    float plantarModeInt = 1.0;
                    msg_motor_plantar.data.clear();
                    msg_motor_plantar.data.push_back(currentTimePercentage);
                    msg_motor_plantar.data.push_back(plantarModeInt);
                    msg_motor_plantar.data.push_back(dirParetic * (plantarPreTension + maxTorquePlantar * pareticTorque));
                    msg_motor_plantar.data.push_back(plantarPosition);
                    msg_motor_plantar.data.push_back(reading.getActualCurrent());
                    msg_motor_plantar.data.push_back(reading.getActualTorque());
                    msg_motor_plantar.data.push_back(reading.getActualPosition());
                    msg_motor_plantar.data.push_back(reading.getActualVelocity());
                    msg_motor_plantar.data.push_back(reading.getBusVoltage());
                    
                    afo_motor_data_plantar.publish(msg_motor_plantar);
                }
                else if (slave->getName() == "NonParetic"){
                    if (setGaitEventNonAffected && setGaitEventAffected){
                        currentTimePercentage = pathPlannerPlantarNonParetic();
                    }
                    if (dorsiRun){
                        command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
                        command.setTargetPosition(plantarNeutralPosition + plantarPosition * dirPlantar);
                        command.setTargetTorque(dirNonParetic * (plantarPreTension + (maxTorqueDorsi) * nonpareticTorque));
                    }
                    else{
                        command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
                        command.setTargetPosition(plantarNeutralPosition);
                        command.setTargetTorque(dirNonParetic * plantarPreTension);
                    }
                    maxon_slave_ptr->stageCommand(command);
                        
                    float dorsiModeInt = 1.0;
                    msg_motor_dorsi.data.clear();
                    msg_motor_dorsi.data.push_back(currentTimePercentage);
                    msg_motor_dorsi.data.push_back(dorsiModeInt);
                    msg_motor_dorsi.data.push_back(dirNonParetic * ((maxTorqueDorsi) * nonpareticTorque + plantarPreTension));
                    msg_motor_dorsi.data.push_back(0);
                    msg_motor_dorsi.data.push_back(reading.getActualCurrent());
                    msg_motor_dorsi.data.push_back(reading.getActualTorque());
                    msg_motor_dorsi.data.push_back(reading.getActualPosition());
                    msg_motor_dorsi.data.push_back(reading.getActualVelocity());
                    msg_motor_dorsi.data.push_back(reading.getBusVoltage());
                    msg_motor_dorsi.data.push_back(dorsiModeInt);
                    msg_motor_dorsi.data.push_back(dorsiStage);
                    
                    afo_motor_data_dorsi.publish(msg_motor_dorsi);
                }
                else {
                    std::cout << slave->getName() << " is not our target device" << std::endl;
                }
            }
        }
    maxonEnabledAfterStartup = true;
            
    next += microseconds(etherCatCommunicationRate);
    std::this_thread::sleep_until(next);
    }							
}

void terminateMotor(int sig)
{
    for(const auto & master: configurator->getMasters())
    {
        master->preShutdown();
    }
abrt = true;
    worker_thread->join();

    for(const auto & master: configurator->getMasters())
    {
        master->shutdown();
    }
std::cout << "Motor Controller Shutdown" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    exit(0);
}

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
 
    afo_gait_nonparetic = n.subscribe("/afo_detector/gait_nonparetic", 1, callbackGaitPhaseNonAffected);
    afo_gait_paretic = n.subscribe("/afo_detector/gait_paretic", 1, callbackGaitPhaseAffected);
    afo_gui_max_torque = n.subscribe("/afo_gui/max_torque", 1, callbackMaxTorque);
    afo_gui_cycle_time = n.subscribe("/afo_gui/cycle_time", 1, callbackCycleTime);
    afo_gui_plantar_run = n.subscribe("/afo_gui/plantar_run", 1, callbackPlantarRun);
ros::Subscriber afo_gui_plantar_trigger_time = n.subscribe("/afo_gui/plantar_trigger_time", 1, callbackPlantarTriggerTime);
ros::Subscriber afo_gui_rand_sub = n.subscribe("/afo_gui/sync", 1, callbackRandRun);
    afo_gui_dorsi_run = n.subscribe("/afo_gui/dorsi_run", 1, callbackDorsiRun);

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
    afo_dorsi_zeroing_done = n.advertise<std_msgs::Bool>("/afo_controller/dorsi_zeroing_done", 10);

    std::signal(SIGINT, terminateMotor);

    configurator = std::make_shared<EthercatDeviceConfigurator>(configPath); 

    // Command Initialize
    dorsiNeutralPosition = 0;
    plantarNeutralPosition = 0;
    dorsiPosition = 0;
    dorsiTorque = 0;
    plantarPosition = 0;
    plantarTorque = 0;
    pareticTorque = 0;
    nonpareticTorque = 0;
    plantarMode = maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode;
    dorsiMode = maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode;

    plantarRun = false;
    dorsiRun = false;
    isRand = false;
    isOFF = false;
    unsigned int paramIdx = 12;
    duration<double> randGap;
    // Load csv file
    auto params = loadRandParamCsv("/home/afo/catkin_ws/src/afo/rand_param.csv");
    //

    
    for(auto & master: configurator->getMasters())
    {
        if(!master->startup())
        {
            std::cerr << "Startup not successful." << std::endl;
            return EXIT_FAILURE;
        }
    }

    worker_thread = std::make_unique<std::thread>(&worker);
    for (int pidx = 0; pidx < 23; pidx++){
        std::cout << params[pidx][0] << ", " << params[pidx][1] << ", " << params[pidx][2] << ", " << std::endl;


    }

    while(ros::ok()){
        if (isRand){
            randGap = high_resolution_clock::now() - timeRand;
            if (randGap.count() > 40.0){
                // Next parameter
                startTime_buff = params[paramIdx][0];
                riseTime_buff = params[paramIdx][1];
                flatTime_buff = params[paramIdx][2];
                if (++paramIdx == 23) paramIdx = 0;
                timeRand = high_resolution_clock::now();
            }
        }
        if (true){
            startTimePF = startTime_buff;
            riseTimePF = riseTime_buff;
            flatTimePF =flatTime_buff;
        }
            std::cout << paramIdx << std::endl;
	    ros::spinOnce();
    }
}
