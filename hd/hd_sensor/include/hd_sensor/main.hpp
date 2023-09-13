#include <iostream>
#include <fstream>
#include "serial.hpp"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"


const char* ID_pcb = "/dev/ttyACM1";