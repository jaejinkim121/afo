#pragma once

#include <iostream>
#include <stdio.h>
#include <libssh/libssh.h>
#include <errno.h>
#include <chrono>
#include <string.h>
#include <thread>
#include <string>

using namespace std;
using namespace std::this_thread;
using namespace std::chrono_literals;
using std::chrono::system_clock;

class syncPi {
private:
	int rc;
	int nbytes;
	char *password;

	ssh_session sync_pi = ssh_new();
	ssh_channel channel;
	bool continue_signal = true;
	bool well_established = false;
public:
	syncPi();
	~syncPi();
	int establish_connect(const char *ip);
	int verify_knownhost();
	int pinModeOutput();
	int remote_gpio(int gpio_num, bool is_high);
	int send_square(int pin, bool is_high);
	int get_endsign();
	int open_channel();
	int close_channel();

};
