#define USE_USBCON
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <Thread.h>


ros::NodeHandle nh;
int pinOut = 0;
int pinIn = A0;
int val = LOW;
int valA = 0;
int preval = LOW;
int counthigh = 0;
int countlow = 0;
int valArray[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
double timeArray[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
ros::Time now;
double now_pre = 0.0;
double now_post = 0.0;
std_msgs::Int32MultiArray msg_val;
std_msgs::Float32MultiArray msg_time;
ros::Publisher pub_val("/afo_arduino/analog_val", &msg_val);
ros::Publisher pub_time("/afo_arduino/analog_time", &msg_time);

Thread analogThread = Thread();

void receiveSync(){
  valA = analogRead(pinIn);
  now = nh.now();
  now_post = now.toSec();

  for (int i=1; i<10;i++){
    valArray[i-1] = valArray[i] + 0;
    timeArray[i-1] = timeArray[i] + 0.0;
  }
  valArray[9] = valA;;
  timeArray[9] = now_post;
  

  
}

void setup(){
  pinMode(pinIn, INPUT);
  pinMode(pinOut, OUTPUT);
  nh.initNode();
  nh.advertise(pub);
  nh.advertise(pub_time);
  msg_val.data.clear();
  msg_time.data.clear();
  // Thread Setup
  analogThread.enabled = true;
  analogThread.onRun(receiveSync);
  analogThread.setInterval(1);
  
}

void loop(){
  msg_val.data.clear();
  msg_time.data.clear();

  for (int j = 0; j < 10; j++){
    msg_val.data.push_back(valArray[j]);
    msg_time.data.push_back(timeArray[j]);
  }
  pub_val.publish(msg_val);
  pub_time.publish(msg_time);
  nh.spinOnce();
}
