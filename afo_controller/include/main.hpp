
#include "ethercat_device_configurator/EthercatDeviceConfigurator.hpp"

#include <maxon_epos_ethercat_sdk/Maxon.hpp>
#include <thread>
#include <chrono>
#include <csignal>
#include "ros/ros.h"
#include "std_msgs/Bool.h"

using namespace std;
using namespace std::chrono;

std::unique_ptr<std::thread> worker_thread;
bool abrt = false;

EthercatDeviceConfigurator::SharedPtr configurator;

unsigned int counter = 0;

double plantarPosition, plantarTorque, plantarMode;
double dorsiPosition, dorsiTorque, dorsiMode;
double plantarNeutralPosition, dorsiNeutralPosition;

bool isPlantar, isDorsi;
system_clock::time_point timeIC, timeOFO, timeFO;

// Configuration
//

// Plantarflexion
double startTime = 0.25;
double endTime = 0.65;
double onTime = endTime - startTime;
double upTimeRatio = 0.75;
double acc = 4 / (upTimeRatio * onTime)^2;
double maxTorque = 0.3; // Nm at lowest level of motor.

// Dorsiflexion
double uptimeDF = 0.1;
double downtimeDF = 0.1;

// To switch target direction easily. CW = 1, CCW = -1
double dirPlantar = 1;
double dirDorsi = -1;
