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
        plantarData = new float[3];
        dorsiData = new float[5];
        gaitPhase = new float[3];

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
        afo_plantar_command_sub = nh->subscribe("/afo_controller/motor_data_plantar", 1, &QNode::callbackPlantar, this);
        afo_dorsi_command_sub = nh->subscribe("/afo_controller/motor_data_dorsi", 1, &QNode::callbackDorsi, this);
        afo_dorsi_zeroing_done_sub = nh->subscribe("/afo_controller/dorsi_zeroing_done", 1, &QNode::callbackDorsiZeroingDone, this);
        afo_gait_paretic_sub = nh->subscribe("/afo_detector/gait_paretic", 1, &QNode::callbackGatiParetic, this);
        afo_gait_nonparetic_sub = nh->subscribe("/afo_detector/gait_nonparetic", 1, &QNode::callbackGatiNonparetic, this);
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

    void QNode::callbackSoleLeft(const std_msgs::Float32MultiArray::ConstPtr& msg){
        soleLeftCnt++;
        if (soleLeftCnt < 9){
            return;
        }
        soleLeftCnt = 0;
        float t = ros::Time::now().toSec() - this->t_begin;
        soleLeftData[0] = t;

        for (int i = 0; i < 6; i++){
            soleLeftData[i+1] = msg->data[i];
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
            soleRightData[i+1] = msg->data[i];
        }
        updateSoleRight();
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
        m.data = r;
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

}