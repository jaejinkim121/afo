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
*/
    // Left Heel & Toe only detection
    if(leftSwing){
        if(d_soleLeft[5] > thLeft[IC][5]){
            leftSwing = false;
        }
    }
    else if (leftToeOff){
        if (d_soleLeft[1] > thLeft[IC][1]){
            leftToeOff = false;
        }
        if (d_soleLeft[3] > thLeft[IC][3]){
            leftToeOff = false;
        }
    }
    else{
        if ((d_soleLeft[1] <  thLeft[FO][1]) & (d_soleLeft[3] < thLeft[FO][3])){
            leftSwing = true;
            leftToeOff = true;
        }
    }
    



    // Right Detection
    /*if(rightSwing){
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

    // Right Heel & Toe only detection
    if(rightSwing){
        if(d_soleright[5] > thright[IC][5]){
            rightSwing = false;
        }
    }
    else if (rightToeOff){
        if (d_soleright[1] > thright[IC][1]){
            rightToeOff = false;
        }
        if (d_soleright[3] > thright[IC][3]){
            rightToeOff = false;
        }
    }
    else{
        if ((d_soleright[1] <  thright[FO][1]) & (d_soleright[3] < thright[FO][3])){
            rightSwing = true;
            rightToeOff = true;
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

void callbackPolyCalib(const std_msgs::Int16MultiArray::ConstPtr& msg){
    initialTimePolycalib = high_resolution_clock::now();
    polySide = msg->data[0];
    polySensor = msg->data[1];
    polyForce = msg->data[2];
    if (polySide == 0){
        savePoly();
        loadPoly();
        runPolycalib = false;
        return;
    }
    runPolycalib = true;
    dataNum = 0;
    for (int i = 0 ;i<6; i++){
        meanLeft[i] = 0;
        meanRight[i] = 0;
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

void updatePoly(){
    if(polySide == 1){
        polyLeft[polyForce][polySensor] = meanLeft[polySensor];
    }
    else if (polySide == 2){
        polyRight[polyForce][polySensor] = meanRight[polySensor];
    }
    else if (polySide == 3){
        for (int i = 0; i<6; i++){
            polyLeft[0][i] = meanLeft[i];
            polyRight[0][i] = meanRight[i];
        }
    }
}

void savePoly(){
    float a, b;
    // poly fitting
    ofstream polyFile;
    std_msgs::Float32MultiArray msg;
    polyFile.open("/home/afo/catkin_ws/src/afo/soleSensor_poly_fit.csv", ios::trunc);

    for (int i =0; i < 6; i++){
        a = referenceForceLow / (polyLeft[1][i] - polyLeft[0][i]);
        b = -polyLeft[0][i] * a;
        polyFile << a << endl << b << endl;
    }
    for (int i = 0; i < 6; i++){
        a = referenceForceLow / (polyRight[1][i] - polyRight[0][i]);
        b = -polyRight[0][i] * a;
        polyFile << a << endl << b << endl;
    }

    /*
    for (int i = 0; i < 6; i++){
        msg.data.clear();
        c = polyLeft[0][i];
        a = (f2 * (polyLeft[1][i] - c) - f1 * (polyLeft[2][i] - c)) / (f1 * f2 * (f1 - f2));
        b = (-f2 * f2 * (polyLeft[1][i] - c) + f1 * f1 * (polyLeft[2][i] - c)) / (f1 * f2 * (f1 - f2));
        polyFile << c << endl << b << endl << a << endl;
    }

    for (int i = 0; i < 6; i++){
        msg.data.clear();
        c = polyRight[0][i];
        a = (f2 * (polyRight[1][i] - c) - f1 * (polyRight[2][i] - c)) / (f1 * f2 * (f1 - f2));
        b = (-f2 * f2 * (polyRight[1][i] - c) + f1 * f1 * (polyRight[2][i] - c)) / (f1 * f2 * (f1 - f2));
        polyFile << c << endl << b << endl << a << endl;
    }
    */
}

void loadPoly(){
    if (!usePolyCalib){
        for (int i = 0; i < 6; i++){
            polyCoeffLeft[1][i] = 1.0;
            polyCoeffRight[1][i] = 1.0;
        }
        return;
    }

    std_msgs::Float32MultiArray msg;
    ifstream polyFile;
    polyFile.open("/home/afo/catkin_ws/src/afo/soleSensor_poly_fit.csv");

    if(!polyFile){
        return;
    }
    string str;
    for (int i = 0; i < 6; i++){
        for (int j = 0; j < 2; j++){
            getline(polyFile, str);
            polyCoeffLeft[j][i] = stof(str);
            msg.data.push_back(polyCoeffLeft[j][i]);
        }
    }
    for (int i = 0; i < 6; i++){
        for (int j = 0; j < 2; j++){
            getline(polyFile, str);
            polyCoeffRight[j][i] = stof(str);
            msg.data.push_back(polyCoeffRight[j][i]);
        }
    }
    afo_poly_fitting_pub.publish(msg);
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
    afo_poly_calib_sub = n.subscribe("/afo_gui/poly_calib", 1, callbackPolyCalib);
    afo_affected_side_sub = n.subscribe("/afo_gui/affected_side", 1, callbackAffectedSide);
    afo_threshold_gap_sub = n.subscribe("/afo_gui/threshold_gap", 1, callbackThresholdGap);
    afo_gait_nonparetic_pub = n.advertise<std_msgs::Int16>("/afo_detector/gait_nonparetic", 100);
    afo_gait_paretic_pub = n.advertise<std_msgs::Int16>("/afo_detector/gait_paretic", 100);
    afo_poly_fitting_pub = n.advertise<std_msgs::Float32MultiArray>("/afo_detector/poly_fit", 100);
    std_msgs::Int16 msg_gait_paretic, msg_gait_nonparetic;

    thresholdSide = LEFT;
    loadThreshold();    
    thresholdSide = RIGHT;
    loadThreshold();    
    loadPoly();

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
            else updateAverage();
        }

        if (runPolycalib){
            currentTimeGap = high_resolution_clock::now() - initialTimePolycalib;
            if(currentTimeGap.count() > recordTimeThreshold){
                runPolycalib = false;
                updatePoly();
            }
            else updateAverage();
        }

	    ros::spinOnce();
        loop_rate.sleep();
    }
}
