
#include "ethercat_device_configurator/EthercatDeviceConfigurator.hpp"

#include <maxon_epos_ethercat_sdk/Maxon.hpp>
#include <thread>
#include <chrono>
#include <csignal>

using namespace std;
using namespace std::chrono;

std::unique_ptr<std::thread> worker_thread;
bool abrt = false;

EthercatDeviceConfigurator::SharedPtr configurator;

unsigned int counter = 0;

double plantarPosition, plantarTorque;
double dorsiPosition, dorsiTorque;
bool isPlantar, isDorsi;
system_clock::time_point timeIC, timeOFO;
