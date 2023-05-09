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

        afo_gui_sole_calibration_pub = nh->advertise<std_msgs::Bool>("/afo_gui/soleCalibration", 100);
        afo_gui_max_torque_pub = nh->advertise<std_msgs::Float32>("/afo_gui/max_torque", 100);
        afo_gui_cycle_time_pub = nh->advertise<std_msgs::Float32>("/afo_gui/cycle_time", 100);
        afo_soleSensor_left_sub = nh->subscribe("/afo_sensor/soleSensor_left", 1, &QNode::callbackSoleLeft, this);
        afo_soleSensor_right_sub = nh->subscribe("/afo_sensor/soleSensor_right", 1, &QNode::callbackSoleRight, this);
        afo_plantar_command_sub = nh->subscribe("/afo_controller/motor_data_plantar", 1, &QNode::callbackPlantar, this);
        afo_dorsi_command_sub = nh->subscribe("/afo_controller/motor_data_dorsi", 1, &QNode::callbackDorsi, this);

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

    void QNode::callbackSoleLeft(const std_msgs::Float32MultiArray::ConstPtr& msg){
        float t = ros::Time::now().toSec() - this->t_begin;
        soleLeftData[0] = t;

        for (int i = 0; i < 6; i++){
            soleLeftData[i+1] = msg->data[i];
        }
        updateSoleLeft();
    }

    void QNode::callbackSoleRight(const std_msgs::Float32MultiArray::ConstPtr& msg){
        float t = ros::Time::now().toSec() - this->t_begin;

        soleRightData[0] = t;

        for (int i = 0; i < 6; i++){
            soleRightData[i+1] = msg->data[i];
        }
        updateSoleRight();
    }

    void QNode::callbackPlantar(const std_msgs::Float32MultiArray::ConstPtr& msg){
        float t = ros::Time::now().toSec() - this->t_begin;

        plantarData[0] = t;
        plantarData[1] = msg->data[2];
        plantarData[2] = msg->data[5];

        updatePlantar();
    }

    void QNode::callbackDorsi(const std_msgs::Float32MultiArray::ConstPtr& msg){
        float t = ros::Time::now().toSec() - this->t_begin;

        dorsiData[0] = t;
        dorsiData[1] = msg->data[2];
        dorsiData[2] = msg->data[5];
        dorsiData[3] = msg->data[3];
        dorsiData[4] = msg->data[6];

        updateDorsi();
    }

}