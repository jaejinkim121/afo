#include "afo_sensor/include/afo_sensor/main.h"

void syncCallback(const std_msgs::BoolConstPtr& msg){
    return 0;    
}

void experimentMarkingCallback(const std_msgs::String::ConstPtr& msg){
    experiment_marking = msg->data.c_str();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "afo_sensor");
	ros::NodeHandle n;
    ros::Subscriber afo_gui_sync_sub = n.subscribe("/afo_gui/sync", 100, syncCallback);
    ros::Subscriber afo_gui_experimentMarking_sub = n.subscribe("/afo_gui/experimentMarking", 100, experimentMarkingCallback);
    ros::Publisher afo_soleSensor_left_pub = n.advertise<std_msgs::Float32MultiArray>("/afo_sensor/soleSensor_left", 100);
    ros::Publisher afo_soleSensor_right_pub = n.advertise<std_msgs::Float32MultiArray>("/afo_sensor/soleSensor_right", 100);
    ros::Publisher afo_imu_pub = n.advertise<std_msgs::Float32MultiArray>("/afo_sensor/imu", 100);

    
    std::time_t t = std::time(0);
	std::tm* now = std::localtime(&t);
	string now_str = to_string(now->tm_mday) + "_" + to_string(now->tm_hour) + "_" + to_string(now->tm_min);

    // 여기에 test_suffix를 ROS에서 받아오는 부분을 추가해서 GUI에서 실험 이름을 넘겨줄 수 있도록 해보자.


    // ------------------------------------------------------------------------------------------------ //

    // Logging File Define
    // GUI 노드로 넘어가야할 가능성이 있음.
    ofstream outFileSoleRight, outFileSoleLeft, outIMU;

	outFileSoleRight.open("../log/data/soleSensor_right_" + now_str + test_suffix + ".csv");
	outFileSoleLeft.open("../log/data/soleSensor_left_" + now_str + test_suffix + ".csv");
	outIMU.open("../log/data/IMU_" + now_str + test_suffix + ".csv");

	outFileSoleLeft.precision(6);
	outFileSoleRight.precision(6);
    outIMU.precision(6);

    outFileSoleLeft.setf(ios_base::fixed, ios_base::floatfield);
	outFileSoleRight.setf(ios_base::fixed, ios_base::floatfield);
	outIMU.setf(ios_base::fixed, ios_base::floatfield);

    // Open Serial Port and attach to correct sensors.
	serialSoleLeft = new serial(ID_leftSole, baudrate);
	serialSoleRight = new serial(ID_rightSole, baudrate);
    serialIMU = new serial(ID_IMU, baudrate);

    // 지금은 sensor node에서 Streaming 신호를 주지만, 나중에는 GUI에서 신호를 줄 수 있도록 해보자.
    leftsole->serialWrite("[s]");
	rightsole->serialWrite("[s]");
    cout << "afo_sensor Node - Sole sensor Streaming Signal Sent" << endl;

    usleep(500000);
 
    while(ros::ok()){
        leftsole->
		ros::spinOnce();
	}
	cout << "afo_sensor Node - ros end - main end" << endl;
    leftsole->serialWrite("[s]")
    rightsole->serialWrite("[s]")
    cout << "afo_sensor Node - Sole sensor Streaming Signal Sent" << endl;

    usleep(500000);
    return 1;
}