// Main Author
/*******************************************************************************
* Jae Jin Kim (gisgen9@snu.ac.kr)

	Reference
	 1) Sudong Lee (slee0@snu.ac.kr)
	 2) Raspberry pi GPIO Code Samples (https://elinux.org/RPi_GPIO_Code_Samples)

*******************************************************************************/
#include "../include/main.h"


void delay(clock_t n) {
	clock_t start = clock();
	while (clock() - start < n);
	
	return;
}

int led(syncPi* sync_pi, serial* leftsole, serial* imu){
	float gyroAbs, leftsoleAbs;
	float th_gyro, th_leftsole;
	th_gyro = 0.3;
	th_leftsole = 2;
	while(!stopsign){
		gyroAbs = imu->get_target_imu();
		leftsoleAbs = leftsole->get_target_sole();

		if(gyroAbs <= th_gyro) sync_pi->remote_gpio(2, true);
		else sync_pi->remote_gpio(2, false);

		if(leftsoleAbs >=th_leftsole) sync_pi->remote_gpio(3, true);
		else sync_pi->remote_gpio(3, false);
	}
	return 1;
}

int init_controller(Controller* afo){
	afo->initiate_control();
	return 1;
}

int toggle_control_trigger(Controller* afo, int p_to_d, int d_to_p){
	afo->controller_trigger(p_to_d, d_to_p);
	return 1;
}

int main(int argc, char** argv){
    int* plantar_parameter = new int[5];
    int* dorsi_parameter = new int[5];

    int rising_time, p_to_d, d_to_p;
    
	p_to_d = 600000;
    d_to_p = 600000;
    
	plantar_parameter[0] = -500;
    plantar_parameter[1] = 2047;
    plantar_parameter[2] = 2047;
    plantar_parameter[3] = 50;
    plantar_parameter[4] = 50;

    dorsi_parameter[0] = 500;
    dorsi_parameter[1] = 2047;
    dorsi_parameter[2] = 2047;
    dorsi_parameter[3] = 50;
    dorsi_parameter[4] = 50;

    rising_time = 400000;

    Controller* afo = new Controller(plantar_parameter, dorsi_parameter, rising_time);
    afo->motor_zero();

    usleep(1000000);

	syncPi* sync_pi = new syncPi();
	
	bool use_sync, current_sync;
	cout << "Do you need to sync? yes - 1, no - 0: ";
	cin >> use_sync;

	if (use_sync) {
		sync_pi->establish_connect(ip_sync_pi);
		int tmp_;
		cout << "Go : " ;
		cin >> tmp_;
		sync_pi->send_square(pin_sync, true);
		current_sync = true;
		cout << "Sync Signal Sended" << endl;
	}

	cout << "********* START *********" << endl;

	chrono::system_clock::time_point start = chrono::system_clock::now();

    thread t_controller_main(init_controller, afo);
    thread t_toggle_trigger(toggle_control_trigger, afo, p_to_d, d_to_p);
	//thread t_ledDisplay(led, sync_pi, leftsole, imu);
    int tmp;
    while(tmp){
    	cin >> tmp;
    	if(tmp == 0){
			sync_pi->remote_gpio(2, false);
	  	  	sync_pi->remote_gpio(3, false);

    		afo->set_stopsign(true);
    		
			stopsign = true;
			break;
    	}
    	if(tmp==1){

    	}
    }

    usleep(100000);

	t_controller_main.join();
	t_toggle_trigger.join();

	//t_ledDisplay.join();

    return 1;
}
