#include <iostream>
#include <string.h>
#include <string>
#include <vector>
#include <sstream>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "rosbag/bag.h"
#include "json.h"

using namespace std;
using namespace std::chrono;

#define FO 0
#define IC 1
#define LEFT true
#define RIGHT false
#define HEELOFF true
// #define VOLTAGE // Uncomment when you want to use voltage-based thresholding.

// Heel은 무조건 1로 해둬야 함 (마지막 element)
//bool isUseIPS[6] = {1, 1, 1, 1, 1, 1}; // MT5, T2, MT3, T1, MT1, H   -->> All use preset
bool isUseIPS[6] = {1, 0, 1, 0, 1, 1}; // -->> MT only preset
//bool isUseIPS[6] = {0, 1, 0, 1, 0, 1}; // -->> Toe only preset
//bool isUseIPS[6] = {1, 1, 0, 0, 1, 1}; // -->> Free use.

void loadThreshold();

float d_soleLeft[7];
float f_soleLeft[7];
float d_soleRight[7];
float f_soleRight[7];
float d_imu[64];

bool is_soleLeft, is_soleRight, is_imu; // ???
bool leftSwing, rightSwing;
bool affectedSide;
bool leftToeOff, rightToeOff;

float thresholdGap[6];
float thLeft[2][6];
float thRight[2][6];
float meanLeft[6];
float meanRight[6];

float ipsCalibrationDataAlpha[2][6][4];
float ipsCalibrationDataConstant[2][6];
float ipsCalibrationDataBP[2][6][3];



int dataNum;

float referenceForceLow = 5.0;
float referenceForceHigh = 10.0;

float recordTimeThreshold = 1.0;
bool runThreshold = false;
bool thresholdSide;

system_clock::time_point initialTimeThreshold, currentTimeThreshold;
system_clock::time_point timeLeftSwing, timeRightSwing;
float swinggap = 0.2;
duration<double> currentTimeGap;

ros::Subscriber afo_soleSensor_left_sub;
ros::Subscriber afo_soleSensor_right_sub;
ros::Subscriber afo_imu_sub;
ros::Subscriber afo_threshold_sub;
ros::Subscriber afo_threshold_update_sub;
ros::Subscriber afo_affected_side_sub;
ros::Subscriber afo_threshold_gap_sub;
ros::Publisher afo_gait_nonparetic_pub;
ros::Publisher afo_gait_paretic_pub;
ros::Publisher afo_ips_force_left_pub;
ros::Publisher afo_ips_force_right_pub;
ros::Publisher afo_zeroing_value_pub;
ros::Publisher afo_threshold_value_pub;
