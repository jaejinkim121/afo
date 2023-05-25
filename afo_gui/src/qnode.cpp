#include <stdlib.h>
#include <iostream>
#include "qnode.hpp"

namespace afo_gui {

    QNode::QNode(int argc, char** argv):
        init_argc(argc),
        init_argv(argv)
        {}

    QNode::~QNode(){
        if(ros::isStarted()) {
            ros::shutdown(); // explicitly needed since we use ros::start();
            ros::waitForShutdown();
        }
        delete nh;
        wait();
    }

    bool QNode::init(){
        soleLeftData = new float[7];
        soleRightData = new float[7];
        soleLeftZero = {0, 0, 0, 0, 0, 0};
        soleRightZero = {0, 0, 0, 0, 0, 0};
        plantarData = new float[3];
        dorsiData = new float[5];
        gaitPhase = new float[3];
        linkX = new double[8];
        linkY = new double[8];
        linkZ = new double[8];
        linkLength = new double[7];
        rz = new double[7];
        pz = new double[7];
        yz = new double[7];

        ros::init(init_argc, init_argv, "afo_gui");
        if ( ! ros::master::check() ) {
            return false;
        }
        init_nh();
        ros::start(); // explicitly needed since our nodehandle is going out of scope.
        
        this->t_begin = ros::Time::now().toSec();

        start();
        return true;
    }

    void QNode::init_nh(){
        nh = new ros::NodeHandle("afo_gui");

        afo_gui_threshold_pub = nh->advertise<std_msgs::Bool>("/afo_gui/run_threshold", 100);
        afo_gui_max_torque_pub = nh->advertise<std_msgs::Float32>("/afo_gui/max_torque", 100);
        afo_gui_cycle_time_pub = nh->advertise<std_msgs::Float32>("/afo_gui/cycle_time", 100);
        afo_gui_max_torque_pub = nh->advertise<std_msgs::Float32>("/afo_gui/max_torque", 100);
        afo_gui_cycle_time_pub = nh->advertise<std_msgs::Float32>("/afo_gui/cycle_time", 100);
        afo_gui_plantar_run_pub = nh->advertise<std_msgs::Bool>("/afo_gui/plantar_run", 100);
        afo_gui_dorsi_run_pub = nh->advertise<std_msgs::Bool>("/afo_gui/dorsi_run", 100);
        afo_gui_streaming_pub = nh->advertise<std_msgs::Bool>("/afo_gui/streaming", 100);

        afo_soleSensor_left_sub = nh->subscribe("/afo_sensor/soleSensor_left", 1, &QNode::callbackSoleLeft, this);
        afo_soleSensor_right_sub = nh->subscribe("/afo_sensor/soleSensor_right", 1, &QNode::callbackSoleRight, this);
        afo_imu_sub = nh->subscribe("/afo_sensor/imu", 1, &QNode::callbackIMU, this);
        afo_plantar_command_sub = nh->subscribe("/afo_controller/motor_data_plantar", 1, &QNode::callbackPlantar, this);
        afo_dorsi_command_sub = nh->subscribe("/afo_controller/motor_data_dorsi", 1, &QNode::callbackDorsi, this);
        afo_dorsi_zeroing_done_sub = nh->subscribe("/afo_controller/dorsi_zeroing_done", 1, &QNode::callbackDorsiZeroingDone, this);
        afo_gait_paretic_sub = nh->subscribe("/afo_detector/gait_paretic", 1, &QNode::callbackGaitParetic, this);
        afo_gait_nonparetic_sub = nh->subscribe("/afo_detector/gait_nonparetic", 1, &QNode::callbackGaitNonparetic, this);
    }

    void QNode::run() {
        int rr;
        nh->getParam("/rr", rr);
        ros::Rate loop_rate(rr);
        while ( ros::ok() ) {
            ros::spinOnce();
            loop_rate.sleep();
        }

        std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
        Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
    }

    float* QNode::getSoleLeftData(){
        return this->soleLeftData;
    }

    float* QNode::getSoleRightData(){
        return this->soleRightData;
    }

    float* QNode::getPlantarData(){
        return this->plantarData;
    }

    float* QNode::getDorsiData(){
        return this->dorsiData;
    }

    float* QNode::getGaitPhase(){
        return this->gaitPhase;
    }

    void QNode::getLink(double* linkX, double* linkY, double* linkZ){
        for (int i = 0; i < 8; i++){
            linkX[i] = this->linkX[i];
            linkY[i] = this->linkY[i];
            linkZ[i] = this->linkZ[i];
        }

        return;
    }

    void QNode::callbackSoleLeft(const std_msgs::Float32MultiArray::ConstPtr& msg){
        soleLeftCnt++;
        if (soleLeftCnt < 9){
            return;
        }
        soleLeftCnt = 0;
        float t = ros::Time::now().toSec() - this->t_begin;
        soleLeftData[0] = t;

        for (int i = 0; i < 6; i++){
            soleLeftData[i+1] = msg->data[i] - soleLeftZero[i];
        }
        updateSoleLeft();
    }

    void QNode::callbackSoleRight(const std_msgs::Float32MultiArray::ConstPtr& msg){
        soleRightCnt++;
        if (soleRightCnt < 9){
            return;
        }
        soleRightCnt = 0;
        float t = ros::Time::now().toSec() - this->t_begin;

        soleRightData[0] = t;

        for (int i = 0; i < 6; i++){
            soleRightData[i+1] = msg->data[i] - soleRightZero[i];
        }
        updateSoleRight();
    }

    void QNode::callbackIMU(const std_msgs::Float32MultiArray::ConstPtr& msg){
        double r[7];
        double p[7];
        double y[7];
        for (int i = 0 ; i < 7 ; i++){
            r[i] = msg->data[9*i];
            p[i] = msg->data[9*i + 1];
            y[i] = msg->data[9*i + 2];
        }

        if (!isIMUZero) {
            for (int i = 0; i<7; i++){
                rz[i] = r[i];
                pz[i] = p[i];
                yz[i] = y[i];
            }
            return;
        }

        Eigen::Vector3d x, y, z, v;
        Eigen::Matrix3d R0, R1;
        x << 1, 0, 0;
        y << 0, 1, 0;
        z << 0, 0, 1;

        linkX[0] = 0;
        linkY[0] = 0;
        linkZ[0] = 0;

        for (int i = 0; i<7; i++){
            R0 = euler2Rotation(rz[i], pz[i], yz[i]);
            R1 = euler2Rotation(r[i], p[i], y[i]);

            switch(i){
                case 0:
                    v = R1 * R0.transpose() * (-x);
                    break;
                case 1:
                    v = R1 * R0.transpose() * z;
                    break;
                case 2:
                    v = R1 * R0.transpose() * z;
                    break;
                case 3:
                    v = R1 * R0.transpose() * (-y);
                    break;
                case 4:
                    v = R1 * R0.transpose() * (-z);
                    break;
                case 5:
                    v = R1 * R0.transpose() * (-z);
                    break;
                case 6:
                    v = R1 * R0.transpose() * x;
            }
            linkX[i+1] = linkX[i] + v[0] * linkLength[i];
            linkY[i+1] = linkY[i] + v[1] * linkLength[i];
            linkZ[i+1] = linkZ[i] + v[2] * linkLength[i];
        }

        double minZ = linkZ[0];

        for (int i = 0; i < 8 ; i++){
            if (linkZ[i] < minZ){
                minZ = linkZ[i];
            }
        }

        for (int i = 0; i < 8 ; i++){
            linkZ[i] = linkZ[i] - minZ;
        }

        if (imuCnt++ > 9){
            imuCnt = 0;
            updateKinematics();
        }
    }

    void QNode::callbackPlantar(const std_msgs::Float32MultiArray::ConstPtr& msg){
        motorPlantarCnt++;
        if (motorPlantarCnt < 9){
            return;
        }
        motorPlantarCnt = 0;
        float t = ros::Time::now().toSec() - this->t_begin;

        plantarData[0] = t;
        plantarData[1] = msg->data[2];
        plantarData[2] = msg->data[5];

        updatePlantar();
    }

    void QNode::callbackDorsi(const std_msgs::Float32MultiArray::ConstPtr& msg){
        motorDorsiCnt++;
        if (motorDorsiCnt < 9){
            return;
        }
        motorDorsiCnt = 0;
        float t = ros::Time::now().toSec() - this->t_begin;

        dorsiData[0] = t;
        dorsiData[1] = msg->data[2];
        dorsiData[2] = msg->data[5];
        dorsiData[3] = msg->data[3];
        dorsiData[4] = msg->data[6];

        updateDorsi();
    }

    void QNode::callbackGaitParetic(const std_msgs::Int16ConstPtr& msg){
        float t = ros::Time::now().toSec() - this->t_begin;
        
        gaitPhase[0] = t;
        gaitPhase[2] = msg->data;
        updateGaitPhase();
    }

    void QNode::callbackGaitNonparetic(const std_msgs::Int16ConstPtr& msg){
        float t = ros::Time::now().toSec() - this->t_begin;
        
        gaitPhase[0] = t;
        gaitPhase[1] = msg->data;
        updateGaitPhase();
    }

    void QNode::callbackDorsiZeroingDone(const std_msgs::BoolConstPtr& msg){
        doneDorsiZeroing();
    }

    void QNode::pubThreshold(bool b){
        std_msgs::Bool m;
        m.data = b;
        this->afo_gui_threshold_pub.publish(m);
    }

    void QNode::pubMaxTorque(float t){
        std_msgs::Float32 m;
        m.data = t;
        afo_gui_max_torque_pub.publish(m);
    }
    
    void QNode::pubCycleTime(float t){
        std_msgs::Float32 m;
        m.data = t;
        afo_gui_cycle_time_pub.publish(m);
    }

    void QNode::pubPlantarRun(bool run){
        std_msgs::Bool m;
        m.data = run;
        afo_gui_plantar_run_pub.publish(m);
    }

    void QNode::pubDorsiRun(bool run){
        std_msgs::Bool m;
        m.data = run;
        afo_gui_dorsi_run_pub.publish(m);
    }

    void QNode::pubStreaming(){
        std_msgs::Bool m;
        m.data = true;
        afo_gui_streaming_pub.publish(m);
    }

    void QNode::imuZeroing(){
        isIMUZero = !isIMUZero;
    }

    void QNode::updateLinkLength(int id, double data){
        loadLinkLength();
        
        ofstream llfile;
        llfile.open("/home/srbl/catkin_ws/src/afo/link_length.csv");

        for (int i = 0; i < 7; i++){
            if (i == id) llfile << data;
            else llfile << linkLength[i];
        }

        linkLength[id] = data;

        llfile.close();
    }
    
    void QNode::loadLinkLength(){
        ifstream llfile;
        llfile.open("/home/srbl/catkin_ws/src/afo/link_length.csv");
        if (!llfile){
            for (int i = 0; i < 7; i++){
                linkLength[i] = 0.5;
            }

            llfile.close();
            return;
        }

        for (int i = 0; i < 7; i++){
            string str;
            getline(llfile, str);
            linkLength[i] = stod(str);
        }
        llfile.close();
        return;
    }

    void QNode::loadSoleZero(int side){
        ifstream thFile;
        if (side == SOLE_LEFT){
            thFile.open("/home/srbl/catkin_ws/src/afo/sole_zero_left.csv");
            for (int i = 0; i < 6; i++){
                std::string str;
                getline(thFile, str);
                soleLeftZero[i] = stof(str);
            }
        }
        else{
            thFile.open("/home/srbl/catkin_ws/src/afo/sole_zero_right.csv");
            for (int i = 0; i < 6; i++){
                std::string str;
                getline(thFile, str);
                soleRightZero[i] = stof(str);
            }
        }
        thFile.close();
    }

}

Eigen::Matrix3d euler2Rotation(const double roll, const double pitch, const double yaw){
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

    Eigen::Matrix3d rotationMatrix = q.matrix();
    return rotationMatrix;
}