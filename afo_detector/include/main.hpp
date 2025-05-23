#include <iostream>
#include <string.h>
#include <string>
#include <vector>
#include <sstream>
#include <chrono>
#include <thread>
#include <cmath>

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "rosbag/bag.h"


using namespace std;
using namespace std::chrono;

#define FO 0
#define IC 1
#define LEFT true
#define RIGHT false

float d_soleLeft[7];
float d_soleRight[7];
float d_imu[64];

bool is_soleLeft, is_soleRight, is_imu; // ???
bool leftSwing, rightSwing;
bool affectedSide;
bool leftToeOff, rightToeOff;


float thresholdGap[4];
float thLeft[2][6];
float thRight[2][6];
float polyLeft[2][6];
float polyRight[2][6];
float polyCoeffLeft[2][6];
float polyCoeffRight[2][6];
float meanLeft[6];
float meanRight[6];
int dataNum;
int polySide, polySensor, polyForce;

float referenceForceLow = 5.0;
float referenceForceHigh = 10.0;

float recordTimeThreshold = 1.0;
bool runThreshold = false;
bool runPolycalib = false;
bool thresholdSide;
bool usePolyCalib = false;

system_clock::time_point initialTimeThreshold, currentTimeThreshold;
system_clock::time_point initialTimePolycalib, currentTimePolycalib;
system_clock::time_point timeLeftSwing, timeRightSwing;
float swinggap = 0.2;
duration<double> currentTimeGap;

ros::Subscriber afo_soleSensor_left_sub;
ros::Subscriber afo_soleSensor_right_sub;
ros::Subscriber afo_imu_sub;
ros::Subscriber afo_threshold_sub;
ros::Subscriber afo_poly_calib_sub;
ros::Subscriber afo_affected_side_sub;
ros::Subscriber afo_threshold_gap_sub;
ros::Publisher afo_gait_nonparetic_pub;
ros::Publisher afo_gait_paretic_pub;
ros::Publisher afo_poly_fitting_pub;


// Function Define
void updatePloy();
void savePoly();
void loadPoly();