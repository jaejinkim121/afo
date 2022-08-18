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


int main(){
	float* sole_norm_t = new float[8];
	ifstream fin("../bin/sole_norm_left.txt");
	char line[255];
	fin.getline(line, sizeof(line));
	fin.close();

	sscanf(line, "%f %f %f %f %f %f %f %f",
			&sole_norm_t[0],&sole_norm_t[1],&sole_norm_t[2],&sole_norm_t[3],
			&sole_norm_t[4],&sole_norm_t[5],&sole_norm_t[6],&sole_norm_t[7]);

	int* plantar_parameter = new int[5];
    int* dorsi_parameter = new int[5];
    int rising_time, p_to_d, d_to_p;

    plantar_parameter[0] = 1000;
    plantar_parameter[1] = 100;
    plantar_parameter[2] = 100;
    plantar_parameter[3] = 100;
    plantar_parameter[4] = 50;

    dorsi_parameter[0] = 1000;
    dorsi_parameter[1] = 100;
    dorsi_parameter[2] = 100;
    dorsi_parameter[3] = 100;
    dorsi_parameter[4] = 50;

    rising_time = 200000;

    string experiment_marking;

	serial * leftsole, *rightsole, *imu;
	syncPi* sync_pi = new syncPi();

	// Get Current Time to name output files.
	std::time_t t = std::time(0);
	std::tm* now = std::localtime(&t);
	string now_str = to_string(now->tm_mday) + "_" + to_string(now->tm_hour) + "_" + to_string(now->tm_min);
	string test_suffix;

	cout << "Which experiment to mark as file name (0 : Null) : ";
	cin >> test_suffix;

	if (test_suffix == "0") {
		test_suffix = "";
	}

	// Make and open output files for whole sensors.
	ofstream outPointRight, outPointLeft, outIMU;

	outPointRight.open("./data/pointSensor_right_" + now_str + test_suffix + ".csv");
	outPointLeft.open("./data/pointSensor_left_" + now_str + test_suffix + ".csv");
	outIMU.open("./data/IMU_" + now_str + test_suffix + ".csv");

	outPointLeft.precision(8);
	outPointLeft.setf(ios_base::fixed, ios_base::floatfield);
	outPointRight.precision(8);
	outPointRight.setf(ios_base::fixed, ios_base::floatfield);
	outIMU.precision(8);
	outIMU.setf(ios_base::fixed, ios_base::floatfield);

	leftsole = new serial(ID_leftSole, baudrate);

	// Load Calibration Data
	bool streaming = false;
	cout <<"Is Streaming?";
	cin >> streaming;
	cout << "test : " << !streaming;

	if(!streaming){
		leftsole->serialWrite("[s]");
	}

	cout << "*** START ***" << endl;

	delay(500);
	chrono::system_clock::time_point start = chrono::system_clock::now();

	thread serialLeftFoot(&serial::readSole, leftsole, std::ref(outPointLeft), start);

    int tmp;
    while(tmp){
    	cin >> tmp;
    	if(tmp == 0){
    		leftsole->get_endsign();
    		break;
    	}
    	if(tmp==1){

    	}
    }

	serialLeftFoot.join();
    return 1;
}
