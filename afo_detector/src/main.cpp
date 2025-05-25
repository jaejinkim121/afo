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

void callbackUpdateThreshold(const std_msgs::BoolConstPtr& msg){
    return;
}



void callbackThresholdGap(const std_msgs::Float32MultiArray::ConstPtr& msg){
    for (int i = 0; i < 4; i++){
        thresholdGap[i] = msg->data[i];
    }
}

float getForcefromVolt(unsigned int side, float voltage, int sensorNum){
    #ifdef VOLT
        return voltage;
    #endif
    float r;
    float bp[4];
    float a[4];

    r = ipsCalibrationDataConstant[side][sensorNum];
    
    for (int i = 0; i<4; i++){
        a[i] = ipsCalibrationDataAlpha[side][sensorNum][i];
    }
    for (int i =0 ; i<3; i++){
        bp[i] = ipsCalibrationDataBP[side][sensorNum][i];
    }

    r += a[0] * voltage;
    r += a[1] * max(voltage - bp[0], 0);
    r += a[2] * max(voltage - bp[1], 0);
    r += a[3] * max(voltage - bp[2], 0);

    if (r<0) return 0;
    
    return r;
}

bool checkForceThreshold(unsigned int side, unsigned int sensorNum, unsigned int isIC){
    float th, f;
    if (side == LEFT){
        f = getForcefromVolt(side, d_soleLeft[sensorNum], sensorNum);
        th = thLEFT[isIC][sensorNum];
        if (isIC) return f >= th;
        else return f < th;
    }
    else {
        f = getForcefromVolt(side, d_soleRight[sensorNum], sensorNum);
        th = thRIGHT[isIC][sensorNum];
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

    ////// ALL SENSOR DETECTION //////
    // Left Detection
    /*
    if(leftSwing){
        for (int i= 0; i<6; i++){
            if(d_soleLeft[i] > thLeft[IC][i]){
                leftSwing = false;
            }
        }
    }
    else{
        leftTmp = true;
        for (int i = 0; i< 6; i++){
            if(d_soleLeft[i] > thLeft[FO][i]){
                leftTmp = false;
                break;
            }
        }
        leftSwing = leftTmp;        
    }
    // Right Detection
    if(rightSwing){
        for (int i= 0; i<6; i++){
            if(d_soleRight[i] > thRight[IC][i]){
                rightSwing = false;
            }
        }
    }
    else{
        rightTmp = true;
        for (int i = 0; i< 6; i++){
            if(d_soleRight[i] > thRight[FO][i]){
                rightTmp = false;
                break;
            }
        }
        rightSwing = rightTmp;        
    }
*/

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
            leftSwing = true;
            leftToeOff = true;
            timeLeftSwing = system_clock::now();
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
            rightSwing = true;
            rightToeOff = true;
            timeRightSwing = system_clock::now();
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
    calibFile.open("/home/afo/catkin_ws/src/afo_detector/sensor_calibration_data.json");

    Json::CharReaderBuilder builder;
	builder["collectComments"] = false;
	Json::Value value;

	JSONCPP_STRING errs;
	bool ok = parseFromStream(builder, calibFile, &value, &errs);

	if (ok == true)
	{
        // LEFT Loading
        for (int i=1; i<=6; i++){
            for (int j=0; j<4; j++){
                ipsCalibrationDataALPHA[LEFT][i-1][j] = value["Left"]["alpha"][(char)i][j];
                cout << "LEFT a " << i << " " << j << " = " << ipsCalibrationDataAlpha[LEFT][i-1][j] << endl;
            }
            
            for (int j=0; j<3;j++){
                ipsCalibrationDataBP[LEFT][i][j] = value["Left"]["breakpoint"][(char)i][j];
                cout << "LEFT b " << i << " " << j << " = " << ipsCalibrationDataBP[LEFT][i-1][j] << endl;
            }

            ipsCalibrationDataConstant[LEFT][i]= value["Left"]["constant"][(char)i];
            cout << "LEFT c = " << ipsCalibrationDataConstant[LEFT][i-1] << end;
        }
        
        // RIGHT Loading
        for (int i=1; i<=6; i++){
            for (int j=0; j<4; j++){
                ipsCalibrationDataALPHA[RIGHT][i-1][j] = value["Right"]["alpha"][(char)i][j];
                cout << "RIGHT a " << i << " " << j << " = " << ipsCalibrationDataAlpha[RIGHT][i-1][j] << endl;
            }
            
            for (int j=0; j<3;j++){
                ipsCalibrationDataBP[RIGHT][i][j] = value["Right"]["breakpoint"][(char)i][j];
                cout << "RIGHT b " << i << " " << j << " = " << ipsCalibrationDataBP[RIGHT][i-1][j] << endl;
            }

            ipsCalibrationDataConstant[RIGHT][i]= value["Right"]["constant"][(char)i];
            cout << "RIGHT c = " << ipsCalibrationDataConstant[RIGHT][i-1] << endl;
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
            thFile << meanLeft[i] + thresholdGap[1+2 * (affectedSide==LEFT)] << endl;
            zeroFile << meanLeft[i] << endl;
        }
        for (int i = 0; i < 6; i++){
            thFile << meanLeft[i] + thresholdGap[2 * affectedSide==LEFT] << endl;
        }
    }
    else {
        thFile.open("/home/afo/catkin_ws/src/afo/threshold_right.csv", ios::trunc);
        zeroFile.open("/home/afo/catkin_ws/src/afo/sole_zero_right.csv", ios::trunc);
        for (int i = 0; i < 6; i++){
            thFile << meanRight[i] + thresholdGap[1 + 2 * (affectedSide==RIGHT)] << endl;
            zeroFile << meanRight[i] << endl;

        }
        for (int i = 0; i < 6 ; i++){
            thFile << meanRight[i] + thresholdGap[2 * affectedSide==RIGHT] << endl;
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
    afo_threshold_update_sub = n.subscribe("/afo_gui/update_threshold", 1, callbackUpdateThreshold);
    afo_affected_side_sub = n.subscribe("/afo_gui/affected_side", 1, callbackAffectedSide);
    afo_threshold_gap_sub = n.subscribe("/afo_gui/threshold_gap", 1, callbackThresholdGap);
    afo_gait_nonparetic_pub = n.advertise<std_msgs::Int16>("/afo_detector/gait_nonparetic", 100);
    afo_gait_paretic_pub = n.advertise<std_msgs::Int16>("/afo_detector/gait_paretic", 100);
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
