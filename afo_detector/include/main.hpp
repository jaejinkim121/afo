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

float thLeft[2][6];
float thRight[2][6];
float meanLeft[6];
float meanRight[6];
int dataNum;

float recordTimeThreshold = 1.0;
bool runThreshold = false;
bool thresholdSide;
system_clock::time_point initialTimeThreshold, currentTimeThreshold;
duration<double> currentTimeGap;
