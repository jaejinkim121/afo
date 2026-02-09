#include "../include/afo_sensor/main.h"

void syncCallback(const std_msgs::BoolConstPtr& msg){
}

void experimentMarkingCallback(const std_msgs::String::ConstPtr& msg){
    experiment_marking = msg->data.c_str();
}

void terminateSensorNode(int sig){
    serialSoleLeft->get_endsign();
    serialSoleRight->get_endsign();
    serialIMU->get_endsign();
    serialSoleLeft->serialWrite("[s]");
    serialSoleRight->serialWrite("[s]");
    cout << "afo_sensor Node - ros end - main end" << endl;
    cout << "afo_sensor Node - Sole sensor Streaming Signal Sent" << endl;

    ros::shutdown();
}

void callbackStreaming(const std_msgs::BoolConstPtr& msg){
    serialSoleLeft->serialWrite("[s]");
    serialSoleRight->serialWrite("[s]");
}

int main(int argc, char** argv){
    signal(SIGINT, terminateSensorNode);

    ros::init(argc, argv, "afo_sensor");
	ros::NodeHandle n;
	int rr;
    n.getParam("/rr", rr);
    ros::Rate loop_rate(rr);
    // ros::Subscriber afo_gui_sync_sub = n.subscribe("/afo_gui/sync", 100, syncCallback);
    // ros::Subscriber afo_gui_experimentMarking_sub = n.subscribe("/afo_gui/experimentMarking", 100, experimentMarkingCallback);
    ros::Subscriber afo_streaming_sub = n.subscribe("/afo_gui/streaming", 1, callbackStreaming);
    ros::Publisher afo_soleSensor_left_pub = n.advertise<std_msgs::Float32MultiArray>("/afo_sensor/soleSensor_left", 100);
    ros::Publisher afo_soleSensor_right_pub = n.advertise<std_msgs::Float32MultiArray>("/afo_sensor/soleSensor_right", 100);
    ros::Publisher afo_imu_pub = n.advertise<std_msgs::Float32MultiArray>("/afo_sensor/imu", 100);

    std::time_t t = std::time(0);
	std::tm* now = std::localtime(&t);
	string now_str = to_string(now->tm_mday) + "_" + to_string(now->tm_hour) + "_" + to_string(now->tm_min);

    // 여기에 test_suffix를 ROS에서 받아오는 부분을 추가해서 GUI에서 실험 이름을 넘겨줄 수 있도록 해보자.
    test_suffix = "";

    // ------------------------------------------------------------------------------------------------ //

    // Logging File Define
    // GUI 노드로 넘어가야할 가능성이 있음.
    ofstream outFileSoleRight, outFileSoleLeft, outIMU;

	outFileSoleRight.open("/home/afo/catkin_ws/src/afo/afo_sensor/log/raw_data/soleSensor_right_" + now_str + test_suffix + ".csv");
	outFileSoleLeft.open("/home/afo/catkin_ws/src/afo/afo_sensor/log/raw_data/soleSensor_left_" + now_str + test_suffix + ".csv");
	outIMU.open("/home/afo/catkin_ws/src/afo/afo_sensor/log/raw_data/IMU_" + now_str + test_suffix + ".csv");

	outFileSoleLeft.precision(6);
	outFileSoleRight.precision(6);
    outIMU.precision(6);

    outFileSoleLeft.setf(ios_base::fixed, ios_base::floatfield);
	outFileSoleRight.setf(ios_base::fixed, ios_base::floatfield);
	outIMU.setf(ios_base::fixed, ios_base::floatfield);
    // Open Serial Port and attach to correct sensors.
	serialSoleLeft = new serial(ID_leftSole, baudrate_sole);
	serialSoleRight = new serial(ID_rightSole, baudrate_sole);
    serialIMU = new serial(ID_IMU, baudrate);

    // 지금은 sensor node에서 Streaming 신호를 주지만, 나중에는 GUI에서 신호를 줄 수 있도록 해보자.

    //serialSoleLeft->serialWrite("[s]");
	//serialSoleRight->serialWrite("[s]");
    cout << "afo_sensor Node - Sole sensor Streaming Signal Sent" << endl;

    usleep(200000);

    chrono::system_clock::time_point start = chrono::system_clock::now();

    thread t_serialLeftFoot(&serial::readSole, serialSoleLeft, std::ref(outFileSoleLeft), start);
	thread t_serialRightFoot(&serial::readSole, serialSoleRight, std::ref(outFileSoleRight), start);
	thread t_serialIMU(&serial::readIMU, serialIMU, std::ref(outIMU), start);
    std_msgs::Float32MultiArray msg_imu;
    std_msgs::Float32MultiArray msg_sole_left;
    std_msgs::Float32MultiArray msg_sole_right;
    std_msgs::Bool msg_sync;

    while(ros::ok()){
        msg_imu.data.clear();
        msg_sole_left.data.clear();
        msg_sole_right.data.clear();
        for ( int i = 0; i<64 ; i++){
            msg_imu.data.push_back(serialIMU->imuData[i]);
        }
        for ( int i = 0 ; i < 7 ; i++){
            msg_sole_left.data.push_back(serialSoleLeft->sole[i]);
            msg_sole_right.data.push_back(serialSoleRight->sole[i]);
        }
        afo_imu_pub.publish(msg_imu);
        afo_soleSensor_left_pub.publish(msg_sole_left);
        afo_soleSensor_right_pub.publish(msg_sole_right);

		ros::spinOnce();
        loop_rate.sleep();
	}
}
