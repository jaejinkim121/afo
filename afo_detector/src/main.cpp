#include "../include/main.hpp"


void callbackSoleLeft(const std_msgs::Float32MultiArray::ConstPtr& msg){
    #ifdef DEBUG
    cout << "Debug - Sole Left data  - ";
    #endif

    for (int i = 0; i< 7; i++){
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

    for (int i = 0; i< 7; i++){
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

    for (int i = 0; i< 64; i++){
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

    // Left Detection
    if(leftSwing){
        for (int i= 0; i<6; i++){
            if(d_soleLeft[i+1] > thLeft[IC][i]){
                leftSwing = false;
            }
        }
    }
    else{
        leftTmp = true;
        for (int i = 0; i< 6; i++){
            if(d_soleLeft[i+1] > thLeft[FO][i]){
                leftTmp = false;
                break;
            }
        }
        leftSwing = leftTmp;        
    }

    // Right Detection
    if(rightSwing){
        for (int i= 0; i<6; i++){
            if(d_soleRight[i+1] > thRight[IC][i]){
                rightSwing = false;
            }
        }
    }
    else{
        rightTmp = true;
        for (int i = 0; i< 6; i++){
            if(d_soleRight[i+1] > thRight[FO][i]){
                rightTmp = false;
                break;
            }
        }
        rightSwing = rightTmp;        
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

void loadThreshold(){
    ifstream thFile;
    bool side = thresholdSide;
    if (side == LEFT){
        thFile.open("/home/srbl/catkin_ws/src/afo/threshold_left.csv");
    }
    else{
        thFile.open("/home/srbl/catkin_ws/src/afo/threshold_right.csv");
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
        thFile.open("/home/srbl/catkin_ws/src/afo/threshold_left.csv", ios::trunc);
        zeroFile.open("/home/srbl/catkin_ws/src/afo/sole_zero_left.csv", ios::trunc);
        for (int i = 0; i < 6; i++){
            thFile << meanLeft[i] + 0.15 << endl;
            zeroFile << meanLeft[i] << endl;

        }
        for (int i = 0; i < 6; i++){
            thFile << meanLeft[i] + 0.10 << endl;
        }
    }
    else {
        thFile.open("/home/srbl/catkin_ws/src/afo/threshold_right.csv", ios::trunc);
        zeroFile.open("/home/srbl/catkin_ws/src/afo/sole_zero_right.csv", ios::trunc);
        for (int i = 0; i < 6; i++){
            thFile << meanRight[i] + 0.12 << endl;
            zeroFile << meanRight[i] << endl;

        }
        for (int i = 0; i < 6 ; i++){
            thFile << meanRight[i] + 0.09 << endl;
        }
    }
    
    thFile.close();
    zeroFile.close();
}

void updateThreshold(){
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
    affectedSide = LEFT;

    // Define ROS
    ros::init(argc, argv, "afo_detector");
    ros::NodeHandle n;
    int rr;
    string configPath;
    n.getParam("/rr", rr);
    ros::Rate loop_rate(rr);
    ros::Subscriber afo_soleSensor_left_sub = n.subscribe("/afo_sensor/soleSensor_left", 1, callbackSoleLeft);
    ros::Subscriber afo_soleSensor_right_sub = n.subscribe("/afo_sensor/soleSensor_right", 1, callbackSoleRight);
    ros::Subscriber afo_imu_sub = n.subscribe("/afo_sensor/imu", 1, callbackIMU);
    ros::Subscriber afo_threshold_sub = n.subscribe("/afo_gui/run_threshold", 1, callbackThreshold);
    ros::Publisher afo_gait_nonparetic_pub = n.advertise<std_msgs::Int16>("/afo_detector/gait_nonparetic", 100);
    ros::Publisher afo_gait_paretic_pub = n.advertise<std_msgs::Int16>("/afo_detector/gait_paretic", 100);
    std_msgs::Int16 msg_gait_paretic, msg_gait_nonparetic;

    thresholdSide = LEFT;
    loadThreshold();    
    thresholdSide = RIGHT;
    loadThreshold();    

    std::cout << "Startup finished" << std::endl;
    int r[4];


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
            else updateThreshold();
        }

	    ros::spinOnce();
        loop_rate.sleep();
    }
}
