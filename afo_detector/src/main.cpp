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

void callbackThresholding(const std_msgs::Bool::ConstPtr& msg){
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

}

// paretic side is left = 0
// paretic side is right = 1
void gaitDetector(int* result){
    string print_arr[2] = {"------------", "------------"};
    result[0] = 0;

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
        result[0] = 1;
        result[1+(affectedSide==LEFT)] = (int)leftSwing + 1;  // 2 when foot-off, 1 when initial contact
    }
    if (rightSwing != prevRight){
        result[0] = 1;
        result[1+(affectedSide==RIGHT)] = (int)rightSwing + 1;  // 2 when foot-off, 1 when initial contact
    }


}

void loadThreshold(){
    ifstream thFile("/home/srbl/catkin_ws/src/afo/threshold.csv");

    if(!thFile){
        cout << "Afo_detector - Cannot open file /home/srbl/catkin_ws/src/afo/threshold.csv - Use New threshold file" << endl;
        for (int i = 0 ; i < 6; i++){
            thLeft[IC][i] = 1.0;
            thLeft[FO][i] = 1.0;
            thRight[IC][i] = 1.0;
            thRight[IC][i] = 1.0;
        }
        thFile.close();
        return;
    }

    for (int i = 0; i<24; i++){
        string str;
        getline(thFile, str);
        if (i<6) thLeft[IC][i%6] = stof(str);
        else if (i<12) thLeft[FO][i%6] = stof(str);
        else if (i<18) thRight[IC][i%6] = stof(str);
        else thRight[FO][i%6] = stof(str);
    }
    thFile.close();
}

void saveThreshold(){
    ofstream thFile("/home/srbl/catkin_ws/src/afo/threshold.csv", ios::trunc);
    if(!thFile){
        cout << "ERROR - afo_detector - Cannot open file /home/srbl/catkin_ws/src/afo/threshold.csv" << endl;
    }
    for (int i = 0; i < 6; i++){
        thFile << meanLeft[i] + 0.05 << endl;
    }
    for (int i = 0; i < 6; i++){
        thFile << meanLeft[i] + 0.1 << endl;
    }
    for (int i = 0; i < 6; i++){
        thFile << meanRight[i] + 0.05 << endl;
    }
    for (int i = 0; i < 6 ; i++){
        thFile << meanRight[i] + 0.1 << endl;
    }

    thFile.close();
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
    string test;
    string configPath;
    n.getParam("/rr", rr);
    n.getParam("/test", test);
    cout << test << endl;
    ros::Rate loop_rate(rr);
    ros::Subscriber afo_soleSensor_left_sub = n.subscribe("/afo_sensor/soleSensor_left", 1, callbackSoleLeft);
    ros::Subscriber afo_soleSensor_right_sub = n.subscribe("/afo_sensor/soleSensor_right", 1, callbackSoleRight);
    ros::Subscriber afo_imu_sub = n.subscribe("/afo_sensor/imu", 1, callbackIMU);
    ros::Subscriber afo_thresholding_sub = n.subscribe("/afo_gui/run_threshold", 1, callbackThresholding);
    ros::Publisher afo_gaitPhase_pub = n.advertise<std_msgs::Int16MultiArray>("/afo_detector/gaitPhase", 100);
    std_msgs::Int16MultiArray msg_gaitPhase;

    loadThreshold();    

    std::cout << "Startup finished" << std::endl;
    int r[3];


    while(ros::ok()){
        msg_gaitPhase.data.clear();
        gaitDetector(r);

        if (r[0] == 1){
            msg_gaitPhase.data.push_back(r[1]);
            msg_gaitPhase.data.push_back(r[2]);
            afo_gaitPhase_pub.publish(msg_gaitPhase);
        }
        
        if (runThreshold){
            currentTimeGap = high_resolution_clock::now() - initialTimeThreshold;
            if (currentTimeGap.count() > 2.0){
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