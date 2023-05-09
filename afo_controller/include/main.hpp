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
int controlMode = EST;
//

// Communication param.
int etherCatCommunicationRate = 5000; // us

// Time Parameter
double periodPreset = 2.0;
double startTime = 0.25;
double endTime = 0.65;
double onTime = endTime - startTime;
double upTimeRatio = 0.75;
double acc = 4 / pow(upTimeRatio * onTime, 2);
double uptimeDF = 0.1;
double downtimeDF = 0.1;
duration<double, micro> eventTimeGap;

// Force Parameter
double maxTorquePlantar = 0.3; // Nm at lowest level of motor.
double maxTorqueDorsi = 0.3;
double maxPositionDorsi = 10;
double positionDiffLimit = 1;
double dorsiZeroingIncrement = 0.1;
double dorsiTorqueSlope = 0.05;
double dorsiPreTension = 0.05;   // It's not normalized value.
double plantarPreTension = 0.05; // It's not normalized value.

// To switch target direction easily. CW = 1, CCW = -1
double dirPlantar = 1;
double dirDorsi = -1;

// Define ros publisher and subscriber
ros::Subscriber afo_gaitPhase;
ros::Publisher afo_motor_data_plantar;
ros::Publisher afo_motor_data_dorsi;
ros::Publisher afo_configuration_maxTorquePlantar;
ros::Publisher afo_configuration_maxTorqueDorsi;
ros::Publisher afo_configuration_startTime;
ros::Publisher afo_configuration_endTime;
ros::Publisher afo_configuration_upTimeRatio;
ros::Publisher afo_configuration_dirPlantar;
ros::Publisher afo_configuration_dorsiNeutralPosition;

std_msgs::Float32MultiArray msg_motor_plantar;
std_msgs::Float32MultiArray msg_motor_dorsi;