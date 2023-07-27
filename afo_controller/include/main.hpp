#include <thread>
#include <chrono>
#include <csignal>
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>

#include "ros/ros.h"
#include "rosbag/bag.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

#include "EthercatDeviceConfigurator.hpp"
#include <maxon_epos_ethercat_sdk/Maxon.hpp>

#define IC 1
#define FO 2
#define EST 3
#define PRE 4

using namespace std;
using namespace std::chrono;

std::unique_ptr<std::thread> worker_thread;
bool abrt = false;

EthercatDeviceConfigurator::SharedPtr configurator;

unsigned int counter = 0;

double plantarPosition, plantarTorque;
double dorsiPosition, dorsiTorque;
double plantarNeutralPosition, dorsiNeutralPosition;
bool setGaitEventAffected = false;
bool setGaitEventNonAffected = false;
bool isDorsiZeroing = false;
maxon::ModeOfOperationEnum plantarMode, dorsiMode;

int dorsiStage = 0;
int dorsiTorqueDir = 1;

bool isPlantar, isDorsi;
system_clock::time_point timeIC, timeOFO, timeFO;

int dorsiBufferFlushingIndex = 0;

// Configuration
int controlMode = PRE;
bool plantarRun, dorsiRun;

//

// Communication param.
int etherCatCommunicationRate = 5000; // us

// Time Parameter
double cycleTime = 2.0 * pow(10,6);
double startTime = 0.25;
double endTime = 0.65;
double onTime = endTime - startTime;
double upTimeRatio = 0.75;
double acc = 4 / pow(upTimeRatio * onTime, 2);
double uptimeDF = 0.1;
double downtimeDF = 0.1;
duration<double, micro> eventTimeGap;

// Force Parameter
double maxTorquePlantar = 0.5; // Nm at lowest level of motor.
double maxTorqueDorsi = 0.5;
double maxPositionDorsi = 3;
double positionDiffLimit = 1;
double dorsiZeroingIncrement = 0.1;
double dorsiTorqueSlope = 0.05;
double dorsiPreTension = 0.06;   // It's not normalized value.
double plantarPreTension = 0.06; // It's not normalized value.

// To switch target direction easily. CW = 1, CCW = -1
double dirPlantar = -1;
double dirDorsi = 1;

// Define ros publisher and subscriber
ros::Subscriber afo_gait_paretic;
ros::Subscriber afo_gait_nonparetic;
ros::Subscriber afo_gui_cycle_time;
ros::Subscriber afo_gui_max_torque;
ros::Subscriber afo_gui_plantar_run;
ros::Subscriber afo_gui_dorsi_run;
ros::Subscriber afo_shutdown_sub;

ros::Publisher afo_motor_data_plantar;
ros::Publisher afo_motor_data_dorsi;
ros::Publisher afo_configuration_cycle_time;
ros::Publisher afo_configuration_maxTorquePlantar;
ros::Publisher afo_configuration_maxTorqueDorsi;
ros::Publisher afo_configuration_maxPositionDorsi;
ros::Publisher afo_configuration_dorsiPreTension;
ros::Publisher afo_configuration_plantarPreTension;
ros::Publisher afo_configuration_startTime;
ros::Publisher afo_configuration_endTime;
ros::Publisher afo_configuration_upTimeRatio;
ros::Publisher afo_configuration_dirPlantar;
ros::Publisher afo_configuration_dorsiNeutralPosition;
ros::Publisher afo_dorsi_zeroing_done;

std_msgs::Float32MultiArray msg_motor_plantar;
std_msgs::Float32MultiArray msg_motor_dorsi;