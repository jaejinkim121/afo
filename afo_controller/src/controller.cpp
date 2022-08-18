#include "../include/controller.h"

Controller::Controller(int* plantar_parameter, int* dorsi_parameter, int rising_time){
    this->sock = this->InitCanInterface("can0");
    this->update_parameter(plantar_parameter, dorsi_parameter, rising_time);
}

Controller::~Controller(){

}

int Controller::motor_zero(){
	int origin = 32767;
	int v0 = 2047;
	int t0 = 2047;
	int p = 0;
	int d = 0;
			uint8_t dorsi_target_frame[CAN_FRAME_MAX_LEN];
		    uint8_t dorsi_release_frame[CAN_FRAME_MAX_LEN];
		    uint8_t plantar_target_frame[CAN_FRAME_MAX_LEN];
		    uint8_t plantar_release_frame[CAN_FRAME_MAX_LEN];
			uint8_t enter_motor[CAN_FRAME_MAX_LEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
			uint8_t exit_motor[CAN_FRAME_MAX_LEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
			uint8_t zero_motor[CAN_FRAME_MAX_LEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};

		    dorsi_target_frame[0] = (origin) >> 8;
		    dorsi_target_frame[1] = (origin) & 0xff;
		    dorsi_target_frame[2] = v0 >> 4;
		    dorsi_target_frame[3] = ((v0 & 0xf) << 4) + (p >> 8);
		    dorsi_target_frame[4] = p & 0xff;
		    dorsi_target_frame[5] = d >> 4;
		    dorsi_target_frame[6] = ((d & 0xf) << 4) + (t0 >> 8);
		    dorsi_target_frame[7] = t0 & 0xff;

		    plantar_target_frame[0] = (origin) >> 8;
		    plantar_target_frame[1] = (origin) & 0xff;
		    plantar_target_frame[2] = v0 >> 4;
		    plantar_target_frame[3] = ((v0 & 0xf) << 4) + (p >> 8);
		    plantar_target_frame[4] = p & 0xff;
		    plantar_target_frame[5] = d >> 4;
		    plantar_target_frame[6] = ((d & 0xf) << 4) + (t0 >> 8);
		    plantar_target_frame[7] = t0 & 0xff;

		    this->TransmitCanFrame(PLANTAR_ID, exit_motor, sizeof(exit_motor));
		    usleep(200000);
		    this->TransmitCanFrame(PLANTAR_ID, zero_motor, sizeof(zero_motor));
		    usleep(200000);
		    this->TransmitCanFrame(PLANTAR_ID, exit_motor, sizeof(exit_motor));
		    usleep(200000);
		    this->TransmitCanFrame(PLANTAR_ID, enter_motor, sizeof(enter_motor));
		    usleep(200000);
		    this->TransmitCanFrame(PLANTAR_ID, plantar_target_frame, sizeof(plantar_target_frame));

		    this->TransmitCanFrame(DORSI_ID, exit_motor, sizeof(exit_motor));
		    usleep(200000);
		    this->TransmitCanFrame(DORSI_ID, zero_motor, sizeof(zero_motor));
		    usleep(200000);
		    this->TransmitCanFrame(DORSI_ID, exit_motor, sizeof(exit_motor));
		    usleep(200000);
		    this->TransmitCanFrame(DORSI_ID, enter_motor, sizeof(enter_motor));
		    usleep(200000);
		    this->TransmitCanFrame(DORSI_ID, dorsi_target_frame, sizeof(dorsi_target_frame));
		    return 1;

}
int Controller::increment(int target){
	int origin = 32767 + target;
	int v0 = 2047;
	int t0 = 2047;
	int p = 100;
	int d = 100;
	uint8_t dorsi_target_frame[8];
	dorsi_target_frame[0] = (origin) >> 8;
	dorsi_target_frame[1] = (origin) & 0xff;
	dorsi_target_frame[2] = v0 >> 4;
	dorsi_target_frame[3] = ((v0 & 0xf) << 4) + (p >> 8);
	dorsi_target_frame[4] = p & 0xff;
	dorsi_target_frame[5] = d >> 4;
	dorsi_target_frame[6] = ((d & 0xf) << 4) + (t0 >> 8);
	dorsi_target_frame[7] = t0 & 0xff;

	this->TransmitCanFrame(DORSI_ID, dorsi_target_frame, sizeof(dorsi_target_frame));
	usleep(1000);
	return 1;
}
int Controller::test_parameter(){
	int origin = 32767;

		uint8_t dorsi_target_frame[CAN_FRAME_MAX_LEN];
	    uint8_t dorsi_release_frame[CAN_FRAME_MAX_LEN];
	    uint8_t plantar_target_frame[CAN_FRAME_MAX_LEN];
	    uint8_t plantar_release_frame[CAN_FRAME_MAX_LEN];
		uint8_t enter_motor[CAN_FRAME_MAX_LEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
		uint8_t exit_motor[CAN_FRAME_MAX_LEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
		uint8_t zero[CAN_FRAME_MAX_LEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};

	    dorsi_target_frame[0] = (origin+this->pos_dorsi) >> 8;
	    dorsi_target_frame[1] = (origin+this->pos_dorsi) & 0xff;
	    dorsi_target_frame[2] = this->vel_dorsi >> 4;
	    dorsi_target_frame[3] = ((this->vel_dorsi & 0xf) << 4) + (this->gain_p_dorsi >> 8);
	    dorsi_target_frame[4] = this->gain_p_dorsi & 0xff;
	    dorsi_target_frame[5] = this->gain_d_dorsi >> 4;
	    dorsi_target_frame[6] = ((this->gain_d_dorsi & 0xf) << 4) + (this->tor_dorsi >> 8);
	    dorsi_target_frame[7] = this->tor_dorsi & 0xff;

	    dorsi_release_frame[0] = origin >> 8;
	    dorsi_release_frame[1] = origin & 0xff;
	    dorsi_release_frame[2] = this->vel_dorsi >> 4;
	    dorsi_release_frame[3] = ((this->vel_dorsi & 0xf) << 4) + (this->gain_p_dorsi >> 8);
	    dorsi_release_frame[4] = this->gain_p_dorsi & 0xff;
	    dorsi_release_frame[5] = this->gain_d_dorsi >> 4;
	    dorsi_release_frame[6] = ((this->gain_d_dorsi & 0xf) << 4) + (this->tor_dorsi >> 8);
	    dorsi_release_frame[7] = this->tor_dorsi & 0xff;

	    plantar_target_frame[0] = (origin+this->pos_plantar) >> 8;
	    plantar_target_frame[1] = (origin+this->pos_plantar) & 0xff;
	    plantar_target_frame[2] = this->vel_plantar >> 4;
	    plantar_target_frame[3] = ((this->vel_plantar & 0xf) << 4) + (this->gain_p_plantar >> 8);
	    plantar_target_frame[4] = this->gain_p_plantar & 0xff;
	    plantar_target_frame[5] = this->gain_d_plantar >> 4;
	    plantar_target_frame[6] = ((this->gain_d_plantar & 0xf) << 4) + (this->tor_plantar >> 8);
	    plantar_target_frame[7] = this->tor_plantar & 0xff;

	    plantar_release_frame[0] = origin >> 8;
	    plantar_release_frame[1] = origin & 0xff;
	    plantar_release_frame[2] = this->vel_plantar >> 4;
	    plantar_release_frame[3] = ((this->vel_plantar & 0xf) << 4) + (this->gain_p_plantar >> 8);
	    plantar_release_frame[4] = this->gain_p_plantar & 0xff;
	    plantar_release_frame[5] = this->gain_d_plantar >> 4;
	    plantar_release_frame[6] = ((this->gain_d_plantar & 0xf) << 4) + (this->tor_plantar >> 8);
	    plantar_release_frame[7] = this->tor_plantar & 0xff;

	    usleep(1000000);
	    cout << "Initiate Control" << endl;

		this->TransmitCanFrame(PLANTAR_ID, plantar_target_frame, sizeof(plantar_target_frame));
		// Wait for Rising Time...
		this->wait_motor(true);
		// Release plantarflexion Command....
		this->TransmitCanFrame(PLANTAR_ID, plantar_release_frame, sizeof(plantar_release_frame));


		this->TransmitCanFrame(DORSI_ID, dorsi_target_frame, sizeof(dorsi_target_frame));
		// Wait for Rising Time...
		this->wait_motor(true);
		// Release dorsiflexion Command....
		this->TransmitCanFrame(DORSI_ID, dorsi_release_frame, sizeof(dorsi_release_frame));
		usleep(1000000);

	    return 1;
}

int Controller::initiate_control(){
    int origin = 32767;

	uint8_t dorsi_target_frame[CAN_FRAME_MAX_LEN];
    uint8_t dorsi_release_frame[CAN_FRAME_MAX_LEN];
    uint8_t plantar_target_frame[CAN_FRAME_MAX_LEN];
    uint8_t plantar_release_frame[CAN_FRAME_MAX_LEN];
	uint8_t enter_motor[CAN_FRAME_MAX_LEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
	uint8_t exit_motor[CAN_FRAME_MAX_LEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
	uint8_t zero[CAN_FRAME_MAX_LEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};

    dorsi_target_frame[0] = (origin+this->pos_dorsi) >> 8;
    dorsi_target_frame[1] = (origin+this->pos_dorsi) & 0xff;
    dorsi_target_frame[2] = this->vel_dorsi >> 4;
    dorsi_target_frame[3] = ((this->vel_dorsi & 0xf) << 4) + (this->gain_p_dorsi >> 8);
    dorsi_target_frame[4] = this->gain_p_dorsi & 0xff;
    dorsi_target_frame[5] = this->gain_d_dorsi >> 4;
    dorsi_target_frame[6] = ((this->gain_d_dorsi & 0xf) << 4) + (this->tor_dorsi >> 8);
    dorsi_target_frame[7] = this->tor_dorsi & 0xff;

    dorsi_release_frame[0] = origin >> 8;
    dorsi_release_frame[1] = origin & 0xff;
    dorsi_release_frame[2] = this->vel_dorsi >> 4;
    dorsi_release_frame[3] = ((this->vel_dorsi & 0xf) << 4) + (this->gain_p_dorsi >> 8);
    dorsi_release_frame[4] = this->gain_p_dorsi & 0xff;
    dorsi_release_frame[5] = this->gain_d_dorsi >> 4;
    dorsi_release_frame[6] = ((this->gain_d_dorsi & 0xf) << 4) + (this->tor_dorsi >> 8);
    dorsi_release_frame[7] = this->tor_dorsi & 0xff;

    plantar_target_frame[0] = (origin+this->pos_plantar) >> 8;
    plantar_target_frame[1] = (origin+this->pos_plantar) & 0xff;
    plantar_target_frame[2] = this->vel_plantar >> 4;
    plantar_target_frame[3] = ((this->vel_plantar & 0xf) << 4) + (this->gain_p_plantar >> 8);
    plantar_target_frame[4] = this->gain_p_plantar & 0xff;
    plantar_target_frame[5] = this->gain_d_plantar >> 4;
    plantar_target_frame[6] = ((this->gain_d_plantar & 0xf) << 4) + (this->tor_plantar >> 8);
    plantar_target_frame[7] = this->tor_plantar & 0xff;

    plantar_release_frame[0] = origin >> 8;
    plantar_release_frame[1] = origin & 0xff;
    plantar_release_frame[2] = this->vel_plantar >> 4;
    plantar_release_frame[3] = ((this->vel_plantar & 0xf) << 4) + (this->gain_p_plantar >> 8);
    plantar_release_frame[4] = this->gain_p_plantar & 0xff;
    plantar_release_frame[5] = this->gain_d_plantar >> 4;
    plantar_release_frame[6] = ((this->gain_d_plantar & 0xf) << 4) + (this->tor_plantar >> 8);
    plantar_release_frame[7] = this->tor_plantar & 0xff;

    cout << "Initiate Control" << endl;

    while(!this->stopsign){
        if(this->trigger_plantar == true){
        	this->trigger_plantar = false;
            // Plantarflexion Command....
            this->TransmitCanFrame(PLANTAR_ID, plantar_target_frame, sizeof(plantar_target_frame));
            // Wait for Rising Time...
            this->wait_motor(true);

            // Release plantarflexion Command....
            this->TransmitCanFrame(PLANTAR_ID, plantar_release_frame, sizeof(plantar_release_frame));
        }

        if(this->trigger_dorsi == true){
        	this->trigger_dorsi = false;
            // Dorsiflexion Command....
            this->TransmitCanFrame(DORSI_ID, dorsi_target_frame, sizeof(dorsi_target_frame));
            // Wait for Rising Time...
            this->wait_motor(true);
            // Release dorsiflexion Command....
            this->TransmitCanFrame(DORSI_ID, dorsi_release_frame, sizeof(dorsi_release_frame));
        }
    }
    this->TransmitCanFrame(PLANTAR_ID, plantar_release_frame, sizeof(plantar_release_frame));
    this->TransmitCanFrame(DORSI_ID, dorsi_release_frame, sizeof(dorsi_release_frame));
    this->TransmitCanFrame(PLANTAR_ID, exit_motor, sizeof(enter_motor));
    this->TransmitCanFrame(DORSI_ID, exit_motor, sizeof(enter_motor));

    cout << "End Controller " << endl;

    return 1;
}

int Controller::update_parameter(int* plantar_parameter, int* dorsi_parameter, int rising_time){
    this->pos_plantar = plantar_parameter[0];
    this->vel_plantar = plantar_parameter[1];
    this->tor_plantar = plantar_parameter[2];
    this->gain_p_plantar = plantar_parameter[3];
    this->gain_d_plantar = plantar_parameter[4];
    this->pos_dorsi = dorsi_parameter[0];
    this->vel_dorsi = dorsi_parameter[1];
    this->tor_dorsi = dorsi_parameter[2];
    this->gain_p_dorsi = dorsi_parameter[3];
    this->gain_d_dorsi = dorsi_parameter[4];
    this->rising_time = rising_time;
    return 1;
}

void Controller::set_trigger(bool trigger_plantar, bool trigger_dorsi){
    this->trigger_plantar = trigger_plantar;
    this->trigger_dorsi = trigger_dorsi;

    return;
}

int Controller::controller_trigger(int p_to_d, int d_to_p){
	while(!this->get_stopsign()){
	    	this->set_trigger(true, false);
	    	usleep(p_to_d);
	    	this->set_trigger(false, true);
	    	usleep(d_to_p);
	    }
	return 1;
}

void Controller::wait_motor(bool is_rising){
    if(is_rising){
        usleep(this->rising_time);
        // sleep rising time.
    }
    else{
        usleep(this->falling_time);
        // sleep fall time.
    }
    return;
}

void Controller::set_stopsign(bool is_stop){
    this->stopsign = is_stop;

    return;
}

bool Controller::get_stopsign(){
	return this->stopsign;
}

int Controller::InitCanInterface(const char *ifname){
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (sock == -1) {
		cout << "Fail to create can socket" << endl;
		return -1;
	}
	cout << "Success to create can socket" << endl;

	struct ifreq ifr;
	strcpy(ifr.ifr_name, ifname);
	int ret = ioctl(sock, SIOCGIFINDEX, &ifr);
	if (ret == -1) {
		perror("Fail to get can interface index -");
		return -1;
	}
	cout << "Success to get can interface index";

	struct sockaddr_can addr;
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	ret = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
	if (ret == -1) {
		perror("Fail to bind can socket -");
		return -1;
	}
	cout << "Success to bind can socket" << endl;

	return sock;
}

int Controller::TransmitCanFrame(const uint32_t id, const uint8_t *data, const size_t data_len){
    struct can_frame frame;
	frame.can_id = id & 0x1fffffff;
//	frame.can_id |= (1 << 31);
	memcpy(frame.data, data, data_len);
	frame.can_dlc = data_len;

	int tx_bytes = write(this->sock, &frame, sizeof(struct can_frame));
	if (tx_bytes == -1) {
		perror("Fail to transmit can frame -");
		return -1;
	}
#ifdef DEBUG_CAN
	cout << "Success to transmit can frame - "<< tx_bytes << "bytes is transmitted" << endl;
#endif
	return 0;
}

int Controller::ReceiveCanFrame(){
    struct can_frame frame;
	while(1){
		cout << "Receiving" << endl;
		int rx_bytes = read(this->sock, &frame, sizeof(struct can_frame));
		if (rx_bytes < 0) {
			perror("Fail to receive can frame -");
			return -1;
		} else if (rx_bytes < (int)sizeof(struct can_frame)) {
			cout << "Incomplete can frame is received - rx_bytes: " << rx_bytes << endl;
			return -1;
		} else if (frame.can_dlc > CAN_FRAME_MAX_LEN) {
			cout << "Invalid dlc: " << frame.can_dlc << endl;
			return -1;
		}

		if (((frame.can_id >> 29) & 1) == 1) {
			cout << "Error frame is received" << endl;
		} else if (((frame.can_id >> 30) & 1) == 1) {
			cout << "RTR frame is received" << endl;
		} else {
			if (((frame.can_id >> 31) & 1) == 1) {
				cout << "11bit long std can frame is received" << endl;
			} else {
				cout << "29bit long std can frame is received" << endl;
				return -1;
			}
		}

	}

	return 0;
}
