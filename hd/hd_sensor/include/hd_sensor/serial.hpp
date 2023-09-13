#pragma once

#include <iostream>
#include <string>
#include <cstring>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <vector>



using namespace std;

class serial {
private:
    int dummy;
    
	int fd;
public:
	int dataEMG[32];
    int dataStrain;
	serial(const char *device, const int baud);
	~serial();
	int serialOpen(const char *device, const int baud);
	void serialWrite(const char*s);
	int serialRead(uint8_t *buffer);
	int serialReadLine(unsigned int limit, uint8_t *buffer);
	int readPCB();

};
