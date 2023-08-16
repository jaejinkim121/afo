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
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>

#define SOLE_LEFT 1
#define SOLE_RIGHT 2
#define MOTOR_PLANTAR 3
#define MOTOR_DORSI 4
#define GAIT_PHASE 5

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
    void callbackIMU(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackPolyFit(const std_msgs::Float32MultiArray::ConstPtr& msg);

    void pubThresholdGap(float* threshold);
    void pubAffectedSide(bool current_affected_side);
    void pubThreshold(bool b);
    void pubMaxTorque(float p, float d);
    void pubCycleTime(float t);
    void pubPlantarRun(bool run);
    void pubDorsiRun(bool run);
    void pubStreaming();
    void pubPolycalib(int side, int num, int force);
    void pubSync(bool sync);
    void imuZeroing();
    void updateLinkLength(int id, double data);
    void loadLinkLength();
    void loadSoleZero(int side);

    float* getSoleLeftData();
    float* getSoleRightData();
    float* getPlantarData();
    float* getDorsiData();
    float* getGaitPhase();
    double getMaxToeClearance(bool isLeft);
    void clearMaxToeClearance();
    double getStride();
    void clearStride();
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
    float* soleLeftZero;
    float* soleRightZero;
    float* plantarData;
    float* dorsiData;
    float* polyFit;
    double* linkX;
    double* linkY;
    double* linkZ;
    double *rz, *pz, *yz;
    double* linkLength;
    float leftToe, rightToe, leftToeMax, rightToeMax;
    float stride;
    double t_begin;

    ros::NodeHandle* nh;

    ros::Publisher afo_gui_affected_side_pub;
    ros::Publisher afo_gui_threshold_gap_pub;
    ros::Publisher afo_gui_threshold_pub;
    ros::Publisher afo_gui_max_torque_pub;
    ros::Publisher afo_gui_cycle_time_pub;
    ros::Publisher afo_gui_plantar_run_pub;
    ros::Publisher afo_gui_dorsi_run_pub;
    ros::Publisher afo_gui_streaming_pub;
    ros::Publisher afo_gui_sync_pub;
    ros::Publisher afo_gui_polycalib;
    ros::Publisher afo_gui_leftToeClearance_pub;
    ros::Publisher afo_gui_rightToeClearance_pub;
    ros::Publisher afo_gui_stride_pub;

    ros::Subscriber afo_soleSensor_left_sub;
    ros::Subscriber afo_soleSensor_right_sub;
    ros::Subscriber afo_imu_sub;
    ros::Subscriber afo_plantar_command_sub;
    ros::Subscriber afo_dorsi_command_sub;
    ros::Subscriber afo_dorsi_zeroing_done_sub;
    ros::Subscriber afo_gait_paretic_sub;
    ros::Subscriber afo_gait_nonparetic_sub;
    ros::Subscriber afo_poly_fit_sub;

    int soleLeftCnt = 0;
    int soleRightCnt = 0;
    int imuCnt = 0;
    int motorPlantarCnt = 0;
    int motorDorsiCnt = 0;
    bool isIMUZero = false;
    
};

}
Eigen::Matrix3d euler2Rotation(double roll, double pitch, double yaw);
#endif