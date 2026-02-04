#define USE_USBCON
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Bool.h>
#include <Thread.h>
#include <ThreadController.h>

ros::NodeHandle nh;
bool flagSwitch = true;
int pinIn = A0;
int pinIn2 = A1;
int pinOut = 0;
int cur_sync = LOW;
int valA = 0;
int valB = 0;
int valArray[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int syncArray[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
std_msgs::Int16MultiArray msg_val;
std_msgs::Int16MultiArray msg_sync;
ros::Publisher pub_val("/afo_arduino/analog_val", &msg_val);
ros::Publisher pub_sync("/afo_arduino/sync_val", &msg_sync);
Thread receiveThread = Thread();
Thread pubThread = Thread();
ThreadController threadControl = ThreadController();

void sync_cb(const std_msgs::Bool& msg){
  if (cur_sync == HIGH) cur_sync = LOW;
  else cur_sync = HIGH;
  digitalWrite(pinOut, cur_sync);

}
ros::Subscriber<std_msgs::Bool> sub_sync("/afo_gui/sync", &sync_cb);

void receiveSync(){
  
  valA = analogRead(pinIn); 
  valB = analogRead(pinIn2);
  
  flagSwitch = true;
  for (int i=1; i<20;i++){
    valArray[i-1] = valArray[i] + 0;
    syncArray[i-1] = syncArray[i] + 0;
  }
  valArray[19] = valA;
  syncArray[19] = valB;
  
  flagSwitch = false;
}

void pubMsg(){
  while(flagSwitch){
    continue;
  }
//  for (int j = 0; j < 10; j++){
  msg_val.data = valArray;
  msg_sync.data = syncArray;
  //}
  pub_val.publish(&msg_val);
  pub_sync.publish(&msg_sync);
  nh.spinOnce();
}

void setup(){
  pinMode(pinIn, INPUT);
  pinMode(pinIn2, INPUT);
  pinMode(pinOut, OUTPUT);
  nh.initNode();
  nh.advertise(pub_val);
  nh.advertise(pub_sync);
  nh.subscribe(sub_sync);
  msg_val.data_length = 20;
  msg_sync.data_length = 20;
  // Thread Setup
  receiveThread.onRun(receiveSync);
  receiveThread.setInterval(1);
  pubThread.onRun(pubMsg);
  pubThread.setInterval(10);
  threadControl.add(&receiveThread);
  threadControl.add(&pubThread);
}

void loop(){
  threadControl.run();  
}
