#include <iostream>
#include <string.h>
#include <vector>
#include <sstream>
#include <chrono>
#include <thread>

#include "serial.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"


using namespace std;

string experiment_marking;
string test_suffix;


// Serial Communication Config.
const char* ID_leftSole = "/dev/ttyACM1";
const char* ID_rightSole = "/dev/ttyACM2";
const char* ID_IMU = "/dev/ttyUSB0";
const int baudrate = 921600;
const int baudrate_sole = 115200;
