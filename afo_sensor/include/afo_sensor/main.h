#include <iostream>
#include <string.h>
#include <vector>
#include <sstream>

#include "afo_sensor/include/afo_sensor/serial.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"


using namespace std;

string experiment_marking;
string test_suffix;
