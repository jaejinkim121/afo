#include "../include/main.hpp"


float getForcefromVolt(unsigned int side, float voltage, int sensorNum){
    #ifdef VOLT
        return voltage;
    #endif
    float r;
    float a,b,c;
    a = ipsCalibrationData[side][sensorNum][0];
    b = ipsCalibrationData[side][sensorNum][1];
    c = ipsCalibrationData[side][sensorNum][2];
    
    r = a * exp(b * voltage) + c;

    if (r<0) return 0;
    
    return r;
}
void callbackSoleLeft(const std_msgs::Float32MultiArray::ConstPtr& msg){
    
    std_msgs::Float32MultiArray msg_force;
    msg_force.data.clear();
    #ifdef DEBUG
    cout << "Debug - Sole Left data  - ";
    #endif

    for (int i = 0; i< 6; i++){
        d_soleLeft[i] = msg->data[i+1];
        f_soleLeft[i] = getForcefromVolt(LEFT, d_soleLeft[i+1], i);
        msg_force.data.push_back(f_soleLeft[i]);
        #ifdef DEBUG
        cout << d_soleLeft[i];
        if (i != 6) cout << ", ";
        #endif

    }

    afo_ips_force_left_pub.publish(msg_force);
    return;
}

void callbackSoleRight(const std_msgs::Float32MultiArray::ConstPtr& msg){
    
    std_msgs::Float32MultiArray msg_force;
    msg_force.data.clear();
    #ifdef DEBUG
    cout << "Debug - Sole Right data  - ";
    #endif

    for (int i = 0; i< 6; i++){
        d_soleRight[i] = msg->data[i+1];
        f_soleRight[i] = getForcefromVolt(RIGHT, d_soleRight[i+1], i);
        msg_force.data.push_back(f_soleRight[i]);
        #ifdef DEBUG
        cout << d_soleRight[i];
        if (i != 6) cout << ", ";
        #endif
        
    }
    afo_ips_force_right_pub.publish(msg_force);
    return;
}

void callbackIMU(const std_msgs::Float32MultiArray::ConstPtr& msg){
    #ifdef DEBUG
    cout << "Debug - IMU data  - ";
    #endif

    for (int i = 0; i< 63; i++){
        d_imu[i] = msg->data[i+1];

        #ifdef DEBUG
        cout << d_imu[i];
        if (i != 63) cout << ", ";
        #endif
        
    }
    return;
}

void callbackThreshold(const std_msgs::Bool::ConstPtr& msg){
    #ifdef DEBUG
    cout << "Debug - thresholding - Initiate Recording";
    #endif

    // Record ## sec data 
    runThreshold = true;
    initialTimeThreshold = high_resolution_clock::now();
    dataNum = 0;
    for (int i = 0 ;i<6; i++){
        meanLeft[i] = 0;
        meanRight[i] = 0;
    }
    thresholdSide = msg->data;

}

void callbackAffectedSide(const std_msgs::BoolConstPtr& msg){
    affectedSide = msg->data;
}

void callbackUpdateThreshold(const std_msgs::BoolConstPtr& msg){
    return;
}

void callbackThresholdGap(const std_msgs::Float32MultiArray::ConstPtr& msg){
    for (int i = 0; i < 6; i++){
        thresholdGap[i] = msg->data[i];
    }

    ifstream iFile;
    ofstream oFile;
    float params[19];
    iFile.open("/home/afo/catkin_ws/src/afo/parameter_list.csv");
    for (int i = 0; i<14; i++){
        string str;
        getline(iFile, str);
        params[i] = stof(str);
    }
    iFile.close();
    for (int i = 0; i < 6; i++){
        params[14+i] = thresholdGap[i];
    }
    oFile.open("/home/afo/catkin_ws/src/afo/parameter_list.csv");
    for (int i = 0; i<20;i++){
        oFile << params[i] << endl;
    }
    oFile.close();

    thresholdSide = LEFT;
    loadThreshold();    
    thresholdSide = RIGHT;
    loadThreshold();

}


bool checkForceThreshold(unsigned int side, unsigned int sensorNum, unsigned int isIC){
    float th, f;
    if (side == LEFT){
        f = getForcefromVolt(side, d_soleLeft[sensorNum], sensorNum);
        th = thLeft[isIC][sensorNum];
        if (isIC) return f >= th;
        else return f < th;
    }
    else {
        f = getForcefromVolt(side, d_soleRight[sensorNum], sensorNum);
        th = thRight[isIC][sensorNum];

        if (isIC) return f >= th;
        else return f < th;
    }
}

// paretic side is left = 0
// paretic side is right = 1
void gaitDetector(int* result){
    result[0] = 0;
    result[1] = 0;
    result[2] = 0;
    result[3] = 0;

    bool leftTmp, rightTmp;
    bool prevLeft, prevRight;
    
    prevLeft = leftSwing;
    prevRight = rightSwing;

    /////// Left Heel & Toe only detection //////
    duration<double> leftDuration, rightDuration;
    // Left detection
    if(leftSwing){
        leftDuration = system_clock::now() - timeLeftSwing;
        if (leftDuration.count() >= swinggap){
            if (checkForceThreshold(LEFT, 5, IC)){
                leftSwing = false;
            }
        }
    }
    else if (leftToeOff){
        if (checkForceThreshold(LEFT, 1, IC)){
            leftToeOff = false;
        }
        if (checkForceThreshold(LEFT, 3, IC)){
            leftToeOff = false;
        }
    }
    else{
        if (checkForceThreshold(LEFT, 1, FO) & checkForceThreshold(LEFT, 3, FO)){
            if ((!HEELOFF) || checkForceThreshold(LEFT, 5, FO)){
                leftSwing = true;
                leftToeOff = true;
                timeLeftSwing = system_clock::now();
            }
        }
    }
    
    // Right Detection    
    if(rightSwing){
        rightDuration = system_clock::now() - timeRightSwing;
        if (rightDuration.count() >= swinggap){
            if (checkForceThreshold(RIGHT, 5, IC)) rightSwing = false;            
        }
    }
    else if (rightToeOff){
        if (checkForceThreshold(RIGHT, 1, IC)){
            rightToeOff = false;
        }
        if (checkForceThreshold(RIGHT, 3, IC)){
            rightToeOff = false;
        }
    }
    else{
        if (checkForceThreshold(RIGHT, 1, FO) & checkForceThreshold(RIGHT, 3, FO)){
            if ((!HEELOFF) || checkForceThreshold(RIGHT, 5, FO)){
                rightSwing = true;
                rightToeOff = true;
                timeRightSwing = system_clock::now();
            }
        }
    }

    if (leftSwing != prevLeft){
        result[affectedSide == LEFT] = 1;
        result[2+(affectedSide==LEFT)] = (int)leftSwing + 1;  // 2 when foot-off, 1 when initial contact
        std::cout << "LEFT Swing : " << leftSwing +1 << std::endl;
    }
    if (rightSwing != prevRight){
        result[affectedSide == RIGHT] = 1;
        result[2+(affectedSide==RIGHT)] = (int)rightSwing + 1;  // 2 when foot-off, 1 when initial contact
        std::cout << "Right Swing : " << rightSwing +1 << std::endl;
    }
}

void loadForceCalibration(){
    ifstream calibFile;
    calibFile.open("/home/afo/catkin_ws/src/afo/afo_detector/sensor_calibration_data.json");

    Json::CharReaderBuilder builder;
	builder["collectComments"] = false;
	Json::Value value;

	JSONCPP_STRING errs;
	bool ok = parseFromStream(builder, calibFile, &value, &errs);
	if (ok == true)
	{
        
        // LEFT Loading
        for (int i=0; i<6; i++){
            ipsCalibrationData[LEFT][i][0] = value["Left"]["a"][to_string(i+1)].asDouble();
            ipsCalibrationData[LEFT][i][1] = value["Left"]["b"][to_string(i+1)].asDouble();
            ipsCalibrationData[LEFT][i][2] = value["Left"]["c"][to_string(i+1)].asDouble();

            ipsCalibrationData[RIGHT][i][0] = value["Right"]["a"][to_string(i+1)].asDouble();
            ipsCalibrationData[RIGHT][i][1] = value["Right"]["b"][to_string(i+1)].asDouble();
            ipsCalibrationData[RIGHT][i][2] = value["Right"]["c"][to_string(i+1)].asDouble();
        }            
	}
	else
	{
		cout << "Parse failed." << endl;
	}
}

void loadThreshold(){
    ifstream thFile;
    bool side = thresholdSide;
    if (side == LEFT){
        thFile.open("/home/afo/catkin_ws/src/afo/sole_zero_left.csv");
    }
    else{
        thFile.open("/home/afo/catkin_ws/src/afo/sole_zero_right.csv");
    }

    if(!thFile){
        for (int i = 0 ; i < 6; i++){
            if (side == LEFT){
                thLeft[IC][i] = 1.0;
                thLeft[FO][i] = 1.0;
            }
            else{
                thRight[IC][i] = 1.0;
                thRight[IC][i] = 1.0;
            }
        }
        thFile.close();
        return;
    }

    for (int i = 0; i<6; i++){
        string str;
        getline(thFile, str);
        if (side == LEFT){
            if (i == 5 && (affectedSide==LEFT)) {
                thLeft[IC][i] = stof(str) + thresholdGap[4];
                thLeft[FO][i] = stof(str) + thresholdGap[5];
            }
            else {
                thLeft[IC][i] = stof(str) + thresholdGap[1+2*(affectedSide==LEFT)];
                thLeft[FO][i] = stof(str) + thresholdGap[2*(affectedSide==LEFT)];
            }
        }
        else{
            if ((i==5) && (affectedSide==RIGHT)) {
                thRight[IC][i] = stof(str) + thresholdGap[4];
                thRight[FO][i] = stof(str) + thresholdGap[5];
            }
            else {
                thRight[IC][i] = stof(str) + thresholdGap[1+2*(affectedSide==RIGHT)];
                thRight[FO][i] = stof(str) + thresholdGap[2*(affectedSide==RIGHT)];
            }
        }
    }
    
    thFile.close();
}

void saveThreshold(){
    bool side = thresholdSide;
    ofstream zeroFile;
    if (side == LEFT){
        zeroFile.open("/home/afo/catkin_ws/src/afo/sole_zero_left.csv", ios::trunc);
        for (int i = 0; i < 6; i++){
//            thFile << meanLeft[i] + thresholdGap[1+2 * (affectedSide==LEFT)] << endl;
            zeroFile << meanLeft[i] << endl;
        }
        for (int i = 0; i < 6; i++){
//            thFile << meanLeft[i] + thresholdGap[2 * affectedSide==LEFT] << endl;
        }
    }
    else {
        zeroFile.open("/home/afo/catkin_ws/src/afo/sole_zero_right.csv", ios::trunc);
        for (int i = 0; i < 6; i++){
//            thFile << meanRight[i] + thresholdGap[1 + 2 * (affectedSide==RIGHT)] << endl;
            zeroFile << meanRight[i] << endl;

        }
        for (int i = 0; i < 6 ; i++){
//            thFile << meanRight[i] + thresholdGap[2 * affectedSide==RIGHT] << endl;
        }
    }
    
    zeroFile.close();
}

void updateAverage(){
    for (int i = 0; i < 6; i++){
        meanLeft[i] += (d_soleLeft[i] - meanLeft[i])/ (dataNum + 1);
        meanRight[i] += (d_soleRight[i] - meanRight[i]) / (dataNum + 1);
    }
    dataNum++;

    return;
}

int main(int argc, char**argv)
{    
    // Initialize values.
    is_soleLeft = false;
    is_soleRight = false;
    is_imu = false;
    leftSwing = false;
    rightSwing = false;

    // Load affected side & thresholdGap
    ifstream paramFile;
    paramFile.open("/home/afo/catkin_ws/src/afo/parameter_list.csv");
    float params[6];

    for (int i = 0; i<20; i++){
        string str;
        getline(paramFile, str);
        if (i < 13) continue;
        params[i-13] = stof(str);
    }

    if (params[0] == 0.0) affectedSide = LEFT;
    else affectedSide = RIGHT;
    
    for (int i = 0; i < 6; i++) thresholdGap[i] = params[i+1];

    // Define ROS
    ros::init(argc, argv, "afo_detector");
    ros::NodeHandle n;
    int rr;
    string configPath;
    n.getParam("/rr", rr);
    ros::Rate loop_rate(rr);
    afo_soleSensor_left_sub = n.subscribe("/afo_sensor/soleSensor_left", 1, callbackSoleLeft);
    afo_soleSensor_right_sub = n.subscribe("/afo_sensor/soleSensor_right", 1, callbackSoleRight);
    afo_imu_sub = n.subscribe("/afo_sensor/imu", 1, callbackIMU);
    afo_threshold_sub = n.subscribe("/afo_gui/run_threshold", 1, callbackThreshold);
    afo_threshold_update_sub = n.subscribe("/afo_gui/update_threshold", 1, callbackUpdateThreshold);
    afo_affected_side_sub = n.subscribe("/afo_gui/affected_side", 1, callbackAffectedSide);
    afo_threshold_gap_sub = n.subscribe("/afo_gui/threshold_gap", 1, callbackThresholdGap);
    afo_gait_nonparetic_pub = n.advertise<std_msgs::Int16>("/afo_detector/gait_nonparetic", 100);
    afo_gait_paretic_pub = n.advertise<std_msgs::Int16>("/afo_detector/gait_paretic", 100);
    afo_ips_force_left_pub = n.advertise<std_msgs::Float32MultiArray>("/afo_detector/soleForce_left", 100);
    afo_ips_force_right_pub = n.advertise<std_msgs::Float32MultiArray>("/afo_detector/soleForce_right", 100);
    
    std_msgs::Int16 msg_gait_paretic, msg_gait_nonparetic;



    thresholdSide = LEFT;
    loadThreshold();    
    thresholdSide = RIGHT;
    loadThreshold();
    loadForceCalibration();

    int r[4];

    timeLeftSwing = system_clock::now();
    timeRightSwing = system_clock::now();
    while(ros::ok()){
        gaitDetector(r);

        if (r[0] == 1){
            msg_gait_nonparetic.data = r[2];
            afo_gait_nonparetic_pub.publish(msg_gait_nonparetic);
        }
        if (r[1] == 1){
            msg_gait_paretic.data = r[3];
            afo_gait_paretic_pub.publish(msg_gait_paretic);
        } 
        
        if (runThreshold){
            currentTimeGap = high_resolution_clock::now() - initialTimeThreshold;
            if (currentTimeGap.count() > recordTimeThreshold){
                runThreshold = false;
                saveThreshold();
                loadThreshold();
            } 
            else updateAverage();
        }

	    ros::spinOnce();
        loop_rate.sleep();
    }
}
