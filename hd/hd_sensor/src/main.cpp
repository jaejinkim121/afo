#include "../include/hd_sensor/main.hpp"

void callbackDummy(const std_msgs::Int32ConstPtr& msg){
    int a = msg->data;
    return;

}

int main(int argc, char** argv){
    ros::init(argc, argv, "hd_sensor");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    serial* pcb = new serial(ID_pcb, 115200);
    ros::Subscriber hd_sensor_sub = n.subscribe("/hd_predictor/dummy", 1, callbackDummy);
    ros::Publisher hd_sensor_pub = n.advertise<std_msgs::Int32MultiArray>("/hd_sensor/dummy_int", 100);


    while(ros::ok()){
        std_msgs::Int32MultiArray msg;
        for (int i = 0; i<32;i++){
            msg.data.push_back(pcb->dataEMG[i]);
        }
        msg.data.push_back(pcb->dataStrain);
        hd_sensor_pub.publish(msg);
	    ros::spinOnce();
        loop_rate.sleep();
    }

}