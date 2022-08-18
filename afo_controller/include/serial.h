#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>
#include <termios.h>
#include <stdio.h>
#include <cstring>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <chrono>
#include <thread>
#include <ctime>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <cmath>

//#define DEBUG


using namespace std;

class serial {
private:
	bool continue_signal = true;
	int fd;
	int test_ind = 1;
	int marker = 0;
	float gyroAbs = 0;
	float soleAbs = 0;
	float gyro[3] = {1,1,1};
	float sole[8] = {1,1,1,1,1,1,1,1};
	float sole_norm[8] = {1,1,1,1,1,1,1,1};


public:
	serial(const char *device, const int baud);
	~serial();
	int serialOpen(const char *device, const int baud);
	void serialWrite(const char*s);
	int serialRead(uint8_t *buffer);
	int serialReadLine(unsigned int limit, uint8_t *buffer);
	int readIMU(ostream& datafile, chrono::system_clock::time_point start);
	int readSole(ostream& datafile, chrono::system_clock::time_point start);
	int get_endsign();
	int change_ind();
	int mark_toggle();
	float get_target_sole();
	float get_target_imu();
	int calculate_target_sole();
	int calculate_target_gyro();
	void set_norm(float* sole_norm_t);

};
