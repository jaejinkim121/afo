#define USE_USBCON
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <Thread.h>
#include <ThreadController.h>

ros::NodeHandle nh;
bool flagSwitch = true;
int pinIn = A0;
int valA = 0;
int valArray[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
std_msgs::Int16MultiArray msg_val;
ros::Publisher pub_val("/afo_arduino/analog_val", &msg_val);

Thread receiveThread = Thread();
Thread pubThread = Thread();
ThreadController threadControl = ThreadController();

void receiveSync(){
  
  valA = analogRead(pinIn); 
  
  flagSwitch = true;
  for (int i=1; i<20;i++){
    valArray[i-1] = valArray[i] + 0;
  }
  valArray[19] = valA;;
  flagSwitch = false;
}

void pubMsg(){
  while(flagSwitch){
    continue;
  }
//  for (int j = 0; j < 10; j++){
  msg_val.data = valArray;
  //}
  pub_val.publish(&msg_val);
  nh.spinOnce();
}

void setup(){
  pinMode(pinIn, INPUT);
  nh.initNode();
  nh.advertise(pub_val);
  
  
  msg_val.data_length = 20;
  
  // Thread Setup
  receiveThread.onRun(receiveSync);
  receiveThread.setInterval(1);
  pubThread.onRun(pubMsg);
  pubThread.setInterval(10);
  threadControl.add(&receiveThread);
  threadControl.add(&pubThread);
}

void loop(){
  //valA = analogRead(pinIn);
  threadControl.run();  
}
