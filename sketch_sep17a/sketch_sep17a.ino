#define USE_USBCON
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <Thread.h>

ros::NodeHandle nh;
bool flagSwitch = true;
int pinIn = A0;
int valA = 0;
int valArray[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float timeArray[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
ros::Time now;
float now_pre = 0.0;
float now_post = 0.0;
std_msgs::Int32MultiArray msg_val;
std_msgs::Float32MultiArray msg_time;
ros::Publisher pub_val("/afo_arduino/analog_val", &msg_val);
ros::Publisher pub_time("/afo_arduino/analog_time", &msg_time);

Thread receiveThread = Thread();
Thread pubThread = Thread();
ThreadController = threadControl = ThreadController();

void receiveSync(){
  
  valA = analogRead(pinIn);
  now = nh.now();
  now_post = now.toSec();

  flagSwitch = true;
  for (int i=1; i<10;i++){
    valArray[i-1] = valArray[i] + 0;
    timeArray[i-1] = timeArray[i] + 0.0;
  }
  valArray[9] = valA;;
  timeArray[9] = now_post;
  flagSwitch = false;
}

void pubMsg(){
  while(flagSwitch){
    continue;
  }
  for (int j = 0; j < 10; j++){
    msg_val.data[j] = valArray[j];
    msg_time.data[j] = timeArray[j];
  }
  pub_val.publish(msg_val);
  pub_time.publish(msg_time);
  nh.spinOnce();
}

void setup(){
  pinMode(pinIn, INPUT);
  pinMode(pinOut, OUTPUT);
  nh.initNode();
  nh.advertise(pub_val);
  nh.advertise(pub_time);
  // Thread Setup
  receiveThread.onRun(receiveSync);
  receiveThread.setInterval(1);
  pubThread.onRun(pubMsg);
  pubThread.setInterval(10);
  threadControl.add(&receiveThread);
  threadControl.add(&pubThread);
  
  msg_val.layout.dimension = 1;
  msg_val.data.length = 10;
  msg_time.layout.dimension = 1;
  msg_time.data.length = 10;
}

void loop(){
  threadControl.run();  
}
