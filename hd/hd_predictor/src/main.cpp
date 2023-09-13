#include "main.hpp"

void callbackDummy(const std_msgs::Int32MultiArray::constPtr& msg){

    for (int i = 0; i < 32; i++){
        emg[i] = msg->data[i];
    }
    strain = msg->data[32];
    isDataUpdated = true;
    return;

}

void inference_one(){
    while(true){
        if(!isDataUpdated) continue;
        isOneRunning = true;
        // main
        inference();
        isOneRunning = false;
    }
}

void inference_two(){
    while(true){
        if(!isDataUpdated) continue;
        if(!isOneRunning) continue;

        inference();
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "hd_predictor");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    ros::Subscriber hd_sensor_sub = n.subscribe("/hd_sensor/dummy", 1, callbackDummy);
    ros::Publisher hd_sensor_pub = n.advertise<std_msgs::Int32>("/hd_predictor/dummy_int", 100);

    thread t_inference_one(inference_one);
	thread t_inference_two(inference_two);

    while(ros::ok()){
        // Inference data publish

        //

	    ros::spinOnce();
        ros::sleep(loop_rate);
    }

    t_inference_one.join();
    t_inference_two.join();
}