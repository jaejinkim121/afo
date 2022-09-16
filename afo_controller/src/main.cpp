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
void send_sync(int fd, bool is_high){
	if (write(fd, "1", int(is_high)) != 1) {
            perror("Error writing to /sys/class/gpio/gpio5/value");
            exit(1);
        }

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
    ros::init(argc, argv, "afo_controlle");
    ros::Rate loop_rate(100);
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

//    Controller* afo = new Controller(plantar_parameter, dorsi_parameter, rising_time);
//    afo->motor_zero();
    // Define sync gpio
int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd == -1) {
        perror("Unable to open /sys/class/gpio/export");
        exit(1);
    }

    if (write(fd, "24", 2) != 2) {
        perror("Error writing to /sys/class/gpio/export");
        exit(1);
    }

    close(fd);

    // Set the pin to be an output by writing "out" to /sys/class/gpio/gpio24/direction

    fd = open("/sys/class/gpio/gpio24/direction", O_WRONLY);
    if (fd == -1) {
        perror("Unable to open /sys/class/gpio/gpio24/direction");
        exit(1);
    }

    if (write(fd, "out", 3) != 3) {
        perror("Error writing to /sys/class/gpio/gpio24/direction");
        exit(1);
    }

    close(fd);

    fd = open("/sys/class/gpio/gpio24/value", O_WRONLY);
    if (fd == -1) {
        perror("Unable to open /sys/class/gpio/gpio24/value");
        exit(1);
    }

	//syncPi* sync_pi = new syncPi();
	
	bool use_sync, current_sync;
        use_sync = true;

	 if (use_sync) {
	 	//sync_pi->establish_connect(ip_sync_pi);
	 	//sync_pi->send_square(pin_sync, true);
	 	current_sync = true;
	 	cout << "Sync Signal Sended" << endl;
	 }

	cout << "********* START *********" << endl;

	chrono::system_clock::time_point start = chrono::system_clock::now();

    //thread t_controller_main(init_controller, afo);
    //thread t_toggle_trigger(toggle_control_trigger, afo, p_to_d, d_to_p);
//    thread t_ledDisplay(led, sync_pi, leftsole, imu);
    int tmp;
    while(ros::ok()){
    	current_sync = !current_sync;
	loop_rate.sleep();

    }

    usleep(100000);

//	t_controller_main.join();
//	t_toggle_trigger.join();


    return 1;
}
