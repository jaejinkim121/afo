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
    void callbackDorsiZeroingDone(const std_msgs::BoolConstPtr& msg);
    void callbackGaitParetic(const std_msgs::Int16ConstPtr& msg);
    void callbackGaitNonparetic(const std_msgs::Int16ConstPtr& msg);

    void pubThreshold(bool b);
    void pubMaxTorque(float t);
    void pubCycleTime(float t);
    void pubPlantarRun(bool run);
    void pubDorsiRun(bool run);
    void pubStreaming();
    void imuZeroing();

    float* getSoleLeftData();
    float* getSoleRightData();
    float* getPlantarData();
    float* getDorsiData();
    float* getGaitPhase();
    void getLink(double* linkX, double* linkY, double* linkZ);


Q_SIGNALS:
    void rosShutdown();
    void updateSoleLeft();
    void updateSoleRight();
    void updateKinematics();
    void updatePlantar();
    void updateDorsi();
    void updateGaitPhase();
    void doneDorsiZeroing();

private:
    int init_argc;
    char** init_argv;

    float* gaitPhase;
    float* soleLeftData;
    float* soleRightData; 
    float* plantarData;
    float* dorsiData;
    double* linkX;
    double* linkY;
    double* linkZ;
    double *rz, *pz, *yz;
    double* linkLength;

    double t_begin;

    ros::NodeHandle* nh;

    ros::Publisher afo_gui_threshold_pub;
    ros::Publisher afo_gui_max_torque_pub;
    ros::Publisher afo_gui_cycle_time_pub;
    ros::Publisher afo_gui_plantar_run_pub;
    ros::Publisher afo_gui_dorsi_run_pub;
    ros::Publisher afo_gui_streaming_pub;

    ros::Subscriber afo_soleSensor_left_sub;
    ros::Subscriber afo_soleSensor_right_sub;
    ros::Subscriber afo_imu_sub;
    ros::Subscriber afo_plantar_command_sub;
    ros::Subscriber afo_dorsi_command_sub;
    ros::Subscriber afo_dorsi_zeroing_done_sub;
    ros::Subscriber afo_gait_paretic_sub;
    ros::Subscriber afo_gait_nonparetic_sub;

    int soleLeftCnt = 0;
    int soleRightCnt = 0;
    int imuCnt = 0;
    int motorPlantarCnt = 0;
    int motorDorsiCnt = 0;
    bool isIMUZero = false;
    
};



}

#endif