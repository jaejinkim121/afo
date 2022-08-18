#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <chrono>
#include <thread>
#include <ctime>
#include <sstream>

#include "controller.h"
#include "serial.h"
#include "sync.h"

using namespace std;
using std::thread;

bool stopsign = false;

chrono::system_clock::time_point start;

// Serial Communication Config.
const char* ID_leftSole = "/dev/ttyACM0";
const char* ID_rightSole = "/dev/ttyACM1";
const char* ID_IMU = "/dev/ttyUSB0";
const int baudrate = 921600;

// Sync Config.
const char* ip_sync_pi = "192.168.0.19";
const int pin_sync = 2;
