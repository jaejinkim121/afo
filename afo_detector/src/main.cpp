#include "../include/main.hpp"


void callbackSoleLeft(const std_msgs::Float32MultiArray::ConstPtr& msg){
    #ifdef DEBUG
    cout << "Debug - Sole Left data  - ";
    #endif

    for (int i = 0; i< 6; i++){
        d_soleLeft[i] = msg->data[i];

        #ifdef DEBUG
        cout << d_soleLeft[i];
        if (i != 6) cout << ", ";
        #endif

    }
    return;
}

void callbackSoleRight(const std_msgs::Float32MultiArray::ConstPtr& msg){
    #ifdef DEBUG
    cout << "Debug - Sole Right data  - ";
    #endif

    for (int i = 0; i< 6; i++){
        d_soleRight[i] = msg->data[i];

        #ifdef DEBUG
        cout << d_soleRight[i];
        if (i != 6) cout << ", ";
        #endif
        
    }
    return;
}

void callbackIMU(const std_msgs::Float32MultiArray::ConstPtr& msg){
    #ifdef DEBUG
    cout << "Debug - IMU data  - ";
    #endif

    for (int i = 0; i< 63; i++){
        d_imu[i] = msg->data[i];

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

void callbackThresholdGap(const std_msgs::Float32MultiArray::ConstPtr& msg){
    for (int i = 0; i < 4; i++){
        thresholdGap[i] = msg->data[i];
    }
}

// paretic side is left = 0
// paretic side is right = 1

void gaitDetector(const std_msgs::Int16MultiArray::ConstPtr& msg){
	std::cout << "gaitDetector run" << std::endl;
	std::cout << "gaitDetector run" << std::endl;
	std::cout << "gaitDetector run" << std::endl;
	std::cout << "gaitDetector run" << std::endl;
	std::cout << "gaitDetector run" << std::endl;
	std::cout << "gaitDetector run" << std::endl;
    result[0] = 0;
    result[1] = 0;

    bool prevSwing;
    prevSwing = rightSwing;
    for (int i = 0; i < 20 ; i++){
        if (msg->data[i] <= 200){
            rightSwing = false;
            break;
        }
        rightSwing = true;
    }
    if (rightSwing!=prevSwing){
        result[affectedSide == RIGHT] = 1;
        result[2+(affectedSide==RIGHT)] = (int)rightSwing + 1;  // 2 when foot-off, 1 when initial contact
        std::cout << "Right Swing : " << rightSwing +1 << std::endl;
        if (rightSwing){
            timeRightSwing = system_clock::now();
        }
        else{
            timeRightStance = system_clock::now();
        }
    }

    if (leftSwing == true){}
    else if ((system_clock::now() - timeRightSwing).count() > oppositeTimeDiff * 10^6){
        leftSwing  = true;
        result[affectedSide == LEFT] = 1;
        result[(affectedSide == LEFT) +2] = (int)leftSwing + 1;
        std::cout << "Left Swing : " << leftSwing + 1 << std::endl;
    }
    if (leftSwing == false){}
    else if ((system_clock::now() - timeRightStance).count() > oppositeTimeDiff * 10^6){
        leftSwing  = false;
        result[affectedSide == LEFT] = 1;
        result[(affectedSide == LEFT) +2] = (int)leftSwing + 1;
        std::cout << "Left Swing : " << leftSwing + 1 << std::endl;
    }


}

void loadThreshold(){
    ifstream thFile;
    bool side = thresholdSide;
    if (side == LEFT){
        thFile.open("/home/afo/catkin_ws/src/afo/threshold_left.csv");
    }
    else{
        thFile.open("/home/afo/catkin_ws/src/afo/threshold_right.csv");
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

    for (int i = 0; i<12; i++){
        string str;
        getline(thFile, str);
        if (side == LEFT){
            if (i<6) thLeft[IC][i] = stof(str);
            else thLeft[FO][i-6] = stof(str);
        }
        else{
            if (i<6) thRight[IC][i] = stof(str);
            else thRight[FO][i-6] = stof(str);
        }
    }

    thFile.close();
}

void saveThreshold(){
    bool side = thresholdSide;
    ofstream thFile, zeroFile;
    if (side == LEFT){
        thFile.open("/home/afo/catkin_ws/src/afo/threshold_left.csv", ios::trunc);
        zeroFile.open("/home/afo/catkin_ws/src/afo/sole_zero_left.csv", ios::trunc);
        for (int i = 0; i < 6; i++){
            thFile << meanLeft[i] + thresholdGap[1+2 * (affectedSide==LEFT)] / polyCoeffLeft[1][i] << endl;
            zeroFile << meanLeft[i] << endl;
        }
        for (int i = 0; i < 6; i++){
            thFile << meanLeft[i] + thresholdGap[2 * affectedSide==LEFT] / polyCoeffLeft[1][i] << endl;
        }
    }
    else {
        thFile.open("/home/afo/catkin_ws/src/afo/threshold_right.csv", ios::trunc);
        zeroFile.open("/home/afo/catkin_ws/src/afo/sole_zero_right.csv", ios::trunc);
        for (int i = 0; i < 6; i++){
            thFile << meanRight[i] + thresholdGap[1 + 2 * (affectedSide==RIGHT)] / polyCoeffRight[1][i] << endl;
            zeroFile << meanRight[i] << endl;

        }
        for (int i = 0; i < 6 ; i++){
            thFile << meanRight[i] + thresholdGap[2 * affectedSide==RIGHT] / polyCoeffRight[1][i] << endl;
        }
    }
    
    thFile.close();
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
    affectedSide = RIGHT;

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
    afo_affected_side_sub = n.subscribe("/afo_gui/affected_side", 1, callbackAffectedSide);
    afo_threshold_gap_sub = n.subscribe("/afo_gui/threshold_gap", 1, callbackThresholdGap);
    afo_gait_nonparetic_pub = n.advertise<std_msgs::Int16>("/afo_detector/gait_nonparetic", 100);
    afo_gait_paretic_pub = n.advertise<std_msgs::Int16>("/afo_detector/gait_paretic", 100);
    ros::Subscriber afo_curexo_sub = n.subscribe("/afo_arduino/analog_val", 1, gaitDetector);

    std_msgs::Int16 msg_gait_paretic, msg_gait_nonparetic;

    thresholdSide = LEFT;
    loadThreshold();    
    thresholdSide = RIGHT;
    loadThreshold();    
 
    std::cout << "Startup finished" << std::endl;
    int r[4];


    while(ros::ok()){
        if (result[0] == 1){
            msg_gait_nonparetic.data = result[2];
            afo_gait_nonparetic_pub.publish(msg_gait_nonparetic);
        }
        if (result[1] == 1){
            msg_gait_paretic.data = result[3];
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
