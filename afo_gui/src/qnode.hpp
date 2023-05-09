#ifndef QNODE_HPP
#define QNODE_HPP

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <QThread>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Point.h>

namespace afo_gui{

class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv);
    virtual ~QNode();
    bool init();
    void init_nh();
    void run();
    
    // callback functions
    void callbackSoleLeft(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackSoleRight(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackPlantar(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackDorsi(const std_msgs::Float32MultiArray::ConstPtr& msg);

    float* getSoleLeftData();
    float* getSoleRightData();
    float* getPlantarData();
    float* getDorsiData();


Q_SIGNALS:
    void rosShutdown();
    void updateSoleLeft();
    void updateSoleRight();
    void updatePlantar();
    void updateDorsi();

private:
    int init_argc;
    char** init_argv;

    float* soleLeftData;
    float* soleRightData; 
    float* plantarData;
    float* dorsiData;

    double t_begin;

    ros::NodeHandle* nh;

    ros::Publisher afo_gui_sole_calibration_pub;
    ros::Publisher afo_gui_max_torque_pub;
    ros::Publisher afo_gui_cycle_time_pub;
    ros::Subscriber afo_soleSensor_left_sub;
    ros::Subscriber afo_soleSensor_right_sub;
    ros::Subscriber afo_plantar_command_sub;
    ros::Subscriber afo_dorsi_command_sub;
};



}

#endif