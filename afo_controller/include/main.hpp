
#include "EthercatDeviceConfigurator.hpp"

#include <maxon_epos_ethercat_sdk/Maxon.hpp>
#include <thread>
#include <chrono>
#include <csignal>
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include "spdlog/spdlog.h"

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

bool isPlantar, isDorsi;
system_clock::time_point timeIC, timeOFO, timeFO;
int tmptmp = 0;
// Configuration
//

// Plantarflexion
double startTime = 0.25;
double endTime = 0.65;
double onTime = endTime - startTime;
double upTimeRatio = 0.75;
double acc = 4 / pow(upTimeRatio * onTime, 2);
double maxTorquePlantar = 1.0; // Nm at lowest level of motor.
double maxTorqueDorsi = 0.2;
double maxPositionDorsi = 10;
double dorsiZeroingIncrement = 0.1;
double dorsiPreTension = 0.1;
double plantarPreTension = 0.1;
// Dorsiflexion
double uptimeDF = 0.1;
double downtimeDF = 0.1;

// To switch target direction easily. CW = 1, CCW = -1
double dirPlantar = -1;
double dirDorsi = 1;
