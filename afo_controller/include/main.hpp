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

double plantarPosition, plantarTorque, plantarCurrentTorque, plantarStopTorque;
double dorsiPosition, dorsiTorque, dorsiCurrentTorque, dorsiStopTorque;
double plantarNeutralPosition, dorsiNeutralPosition;
bool setGaitEventAffected = false;
bool setGaitEventNonAffected = false;
bool isDorsiZeroing = false;
bool forced_trigger = false;
maxon::ModeOfOperationEnum plantarMode, dorsiMode;

int dorsiStage = 0;
int dorsiTorqueDir = 1;

bool isPlantar, isDorsi;
bool setDF_cue_MH, setPF_cue_MH;

system_clock::time_point timeIC, timeOFO, timeFO, timeCuePF, timeCueDF;
system_clock::time_point timeFT;

int dorsiBufferFlushingIndex = 0;

// Configuration
int controlMode = PRE;
bool plantarRun, dorsiRun;

//

// Communication param.
int etherCatCommunicationRate = 5500; // us

// Time Parameter
double cycleTime = 1.2 * pow(10,6);
double startTimePF = 0.1;
double riseTimePF = 0.3;
double fallTimePF = 0.1;
double flatTimePF = 0.2;
double endTimePF = startTimePF + riseTimePF + fallTimePF + flatTimePF;
double relaxTime = 0.1;
double startTimeDF = 0.0;
double riseTimeDF = 0.05;
double fallTimeDF = 3.0;
double stance_time = 0.65;
double trigger_layback_ms = 0.0;
duration<double, micro> eventTimeGap;

// Force Parameter
double maxTorquePlantar = 0.3; // Nm at lowest level of motor.
double maxTorqueDorsi = 0.1;
double maxPositionDorsi = 3000;
double positionDiffLimit = 500;
double dorsiZeroingIncrement = 0.1;
double dorsiTorqueSlope = 0.05;
double dorsiPreTension = 0.025;   // It's not normalized value.
double plantarPreTension = 0.025; // It's not normalized value.

// To switch target direction easily. CW = -1, CCW = 1
double dirPlantar = 1;
double dirDorsi = -1;

// Define ros publisher and subscriber
ros::Subscriber afo_gait_paretic;
ros::Subscriber afo_gait_nonparetic;
ros::Subscriber afo_gui_cycle_time;
ros::Subscriber afo_gui_max_torque;
ros::Subscriber afo_gui_plantar_run;
ros::Subscriber afo_gui_dorsi_run;
ros::Subscriber afo_shutdown_sub;
ros::Subscriber afo_gui_mh_df_run;
ros::Subscriber afo_gui_mh_pf_run;
ros::Subscriber afo_gui_forced_trigger;
ros::Subscriber afo_mw_forced_trigger;

ros::Publisher afo_motor_data_plantar;
ros::Publisher afo_motor_data_dorsi;
ros::Publisher afo_configuration_cycle_time;
ros::Publisher afo_configuration_maxTorquePlantar;
ros::Publisher afo_configuration_maxTorqueDorsi;
ros::Publisher afo_configuration_riseTimePlantar;
ros::Publisher afo_configuration_riseTimeDorsi;
ros::Publisher afo_configuration_fallTimePlantar;
ros::Publisher afo_configuration_fallTimeDorsi;
ros::Publisher afo_configuration_startTimePlantar;
ros::Publisher afo_configuration_startTimeDorsi;
ros::Publisher afo_configuration_maxPositionDorsi;
ros::Publisher afo_configuration_dorsiPreTension;
ros::Publisher afo_configuration_plantarPreTension;
ros::Publisher afo_configuration_dirPlantar;
ros::Publisher afo_configuration_dorsiNeutralPosition;
ros::Publisher afo_dorsi_zeroing_done;

std_msgs::Float32MultiArray msg_motor_plantar;
std_msgs::Float32MultiArray msg_motor_dorsi;
