#include <iostream>
#include <fstream>
#include "serial.hpp"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"

using namespace std;

    int emg[32];
    int strain;
    bool isDataUpdated = false;
    bool isOneRunning = false;