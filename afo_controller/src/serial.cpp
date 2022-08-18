#include "../include/serial.h"

void _delay(clock_t n) {
	clock_t start = clock();
	while (clock() - start < n);
}

serial::serial(const char *device, const int baud) {
	this->serialOpen(device, baud);
}

serial::~serial() {

}

int serial::serialOpen(const char *device, const int baud) {
	struct termios options;
	speed_t myBaud;
	int     status, fd;

	switch (baud)
	{
	case      50:	myBaud = B50; break;
	case      75:	myBaud = B75; break;
	case     110:	myBaud = B110; break;
	case     134:	myBaud = B134; break;
	case     150:	myBaud = B150; break;
	case     200:	myBaud = B200; break;
	case     300:	myBaud = B300; break;
	case     600:	myBaud = B600; break;
	case    1200:	myBaud = B1200; break;
	case    1800:	myBaud = B1800; break;
	case    2400:	myBaud = B2400; break;
	case    4800:	myBaud = B4800; break;
	case    9600:	myBaud = B9600; break;
	case   19200:	myBaud = B19200; break;
	case   38400:	myBaud = B38400; break;
	case   57600:	myBaud = B57600; break;
	case  115200:	myBaud = B115200; break;
	case  230400:	myBaud = B230400; break;
	case  460800:	myBaud = B460800; break;
	case  500000:	myBaud = B500000; break;
	case  576000:	myBaud = B576000; break;
	case  921600:	myBaud = B921600; break;
	case 1000000:	myBaud = B1000000; break;
	case 1152000:	myBaud = B1152000; break;
	case 1500000:	myBaud = B1500000; break;
	case 2000000:	myBaud = B2000000; break;
	case 2500000:	myBaud = B2500000; break;
	case 3000000:	myBaud = B3000000; break;
	case 3500000:	myBaud = B3500000; break;
	case 4000000:	myBaud = B4000000; break;

	default:
		return -2;
	}

	if ((fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
		return -1;

	fcntl(fd, F_SETFL, O_RDWR);

	// Get and modify current options:

	tcgetattr(fd, &options);

	cfmakeraw(&options);
	cfsetispeed(&options, myBaud);
	cfsetospeed(&options, myBaud);

	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_oflag &= ~OPOST;

	options.c_cc[VMIN] = 0;
	options.c_cc[VTIME] = 100;	// Ten seconds (100 deciseconds)

	tcsetattr(fd, TCSANOW, &options);

	ioctl(fd, TIOCMGET, &status);

	status |= TIOCM_DTR;
	status |= TIOCM_RTS;

	ioctl(fd, TIOCMSET, &status);

	usleep(10000);	// 10mS

	this->fd = fd;
	
	return 1;

}

void serial::serialWrite(const char *s) {
	write(this->fd, s, strlen(s));
	return;
}

int serial::serialRead(uint8_t *buffer) {
	if (read(this->fd, buffer, 1) != 1) return -1;
	
	return 1;

}

int serial::serialReadLine(unsigned int limit, uint8_t *buffer) {

	uint8_t ch = 0;
	int bytesRead = 0;
	for (int i = 0; i < limit; i++)
	{
		bytesRead += serialRead(&ch);
		buffer[i] = ch;

		if ((char)ch == '\n') break;

	}

	return bytesRead;
}

int serial::readIMU(ostream& datafile, chrono::system_clock::time_point start) {
	uint8_t incomingData[255];
	uint8_t trimData[255];

	cout.precision(8);
	cout << fixed;
	int nread;
	int num = 0;
	chrono::duration<double> sec;
	double secD;
	while (this->continue_signal) {
		nread = this->serialReadLine(255, incomingData);
		if (nread > 0) {
			incomingData[nread - 2] = 0;
			sec = chrono::system_clock::now() - start;
			secD = sec.count();
			datafile << this->marker << ", " << this->test_ind << "," << secD << ", " << incomingData << endl;

			num = 0;
			if(incomingData[4] != '5') continue;
			for(int i = 0; i<255; i++){
					if(incomingData[i] == ','){
						num++;
					}
					if(num == 7){
						num = i+1;
						break;
					}
			}
			for (int i = 0; i<255; i++){
				trimData[i] = incomingData[num];
				num++;
				if(num == nread-2) break;
			}
			sscanf((char*)trimData, "%f,%f,%f", &this->gyro[0], &this->gyro[1], &this->gyro[2]);
			this->calculate_target_gyro();

		}

	}
	cout << "IMU Serial Reading End" << endl;
	return 1;
}


int serial::readSole(ostream& datafile, chrono::system_clock::time_point start) {
	uint8_t incomingData[255];
	
	cout.precision(8);
	cout << fixed;
	int nread;
	int num = 0;
	chrono::duration<double> sec;
	double secD;
	while (this->continue_signal) {
		nread = this->serialReadLine(255, incomingData);
		if (nread > 0) {
			incomingData[nread - 2] = 0;
			sec = chrono::system_clock::now() - start;
			secD = sec.count();
			datafile << this->marker << ", " << this->test_ind << "," << secD << ", " << incomingData << endl;
			sscanf((char*)incomingData, "%f %f %f %f %f %f %f %f", &sole[0], &sole[1], &sole[2], &sole[3], &sole[4], &sole[5], &sole[6], &sole[7]);
			this->calculate_target_sole();
		}

	}
	cout << "Sole Serial Reading End" << endl;
	return 1;
}

int serial::get_endsign() {
	this->continue_signal = false;
	return 1;
}

int serial::mark_toggle(){
	if(this->marker == 0){
		this->marker = 1;
	}
	else if (this->marker == 1){
		this->marker = 0;
	}
	return 1;
}

int serial::change_ind(){
	if(this->test_ind == 1){
		this->test_ind = 0;
	}
	else if (this->test_ind == 0 ){
		this->test_ind = 1;
	}
	return 1;
}

float serial::get_target_sole(){
	return this->soleAbs;
}

float serial::get_target_imu(){
	return this->gyroAbs;
}

int serial::calculate_target_sole(){
	float r;
	for (int i = 0 ; i < 8 ; i++){
		r += (sole[i] -1) * sole_norm[i];
	}
	this->soleAbs = r;

	return 1;
}

int serial::calculate_target_gyro(){
	float g0, g1, g2;
	g0 = this->gyro[0];
	g1 = this->gyro[1];
	g2 = this->gyro[2];
	this->gyroAbs = sqrt(g0*g0 + g1*g1 + g2*g2);
	return 1;
}

void serial::set_norm(float* sole_norm_t){
	for (int i = 0; i<8 ; i++){
		this->sole_norm[i] = sole_norm_t[i] / 100.0;
	}
	return;
}

