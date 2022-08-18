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
}

int init_controller(Controller* afo){
	afo->initiate_control();
	return 1;
}

int controller_trigger(Controller* afo, int p_to_d, int d_to_p){
	while(!stopsign){
	    	afo->set_trigger(true, false);
	    	cout << "p trigger" << endl;
	    	usleep(p_to_d);
	    	afo->set_trigger(false, true);
	    	cout << "d trigger" << endl;
	    	usleep(d_to_p);
	    }
	return 1;

}

int main(){

    int* plantar_parameter = new int[5];
    int* dorsi_parameter = new int[5];
    int rising_time, p_to_d, d_to_p;
    p_to_d = 4000000;
    d_to_p = 4000000;
    plantar_parameter[0] = -100;
    plantar_parameter[1] = 2047;
    plantar_parameter[2] = 2047;
    plantar_parameter[3] = 100;
    plantar_parameter[4] = 100;

    dorsi_parameter[0] = 100;
    dorsi_parameter[1] = 2047;
    dorsi_parameter[2] = 2047;
    dorsi_parameter[3] = 100;
    dorsi_parameter[4] = 100;

    rising_time = 2000000;

    Controller* afo = new Controller(plantar_parameter, dorsi_parameter, rising_time);
    afo->motor_zero();
    int tmp;
    int value;
    while(true){

    	cout << "Which parameter do you want to change?" << endl;
    	cin >> tmp;
    	cout << "Enter Value for that value:";
    	cin >> value;
    	switch(tmp){
    	case 0:
    		return 1;
    		break;
    	case 1:
    	    		plantar_parameter[0] = value;
    	    		break;
    	case 2:
    	    		plantar_parameter[1] = value;
    	    		break;
    	case 3:
    	    		plantar_parameter[2] = value;
    	    		break;
    	case 4:
    	    		plantar_parameter[3] = value;
    	    		break;
    	case 5:
    	    		plantar_parameter[4] = value;
    	    		break;
    	case 6:
    		dorsi_parameter[0] = value;
    		break;
    	case 7:
    		dorsi_parameter[1] = value;
    		break;
    	case 8:
    		dorsi_parameter[2] = value;
    		break;
    	case 9:
    		dorsi_parameter[3] = value;
    		break;
    	case 10:
    		dorsi_parameter[4] = value;
    		break;
    	case 11:
    		rising_time = value;
    		break;
    	}
    	afo->update_parameter(plantar_parameter, dorsi_parameter, rising_time);
    	usleep(100000);
    	afo->test_parameter();


    }
    return 1;
}

