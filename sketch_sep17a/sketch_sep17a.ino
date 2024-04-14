#define USE_USBCON
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;
int pinOut = 0;
int pinIn = A0;
int val = LOW;
int valA = 0;
int preval = LOW;
int counthigh = 0;
int countlow = 0;
ros::Time now;
double now_pre = 0.0;
double now_post = 0.0;
//
//void callBack(const std_msgs::Bool& msg){
//  if (msg.data == true){
//    digitalWrite(pinOut, HIGH);
//  }
//  else{
//    digitalWrite(pinOut, LOW);
//  }
//  
//}

std_msgs::Bool msg;
std_msgs::Float32 msg_time;
//ros::Subscriber<std_msgs::Bool> sub("/afo_predictor/sync_pred", callBack);
ros::Publisher pub("/afo_arduino/forced_trigger", &msg);
ros::Publisher pub_time("/afo_arduino/sync_time", &msg_time);
void setup(){
  pinMode(pinIn, INPUT);
  pinMode(pinOut, OUTPUT);
  nh.initNode();
  nh.advertise(pub);
  nh.advertise(pub_time);
  
}

void loop(){
  valA = analogRead(pinIn);
  if (valA > 900){
    val = HIGH;
  }
  else{
    val = LOW;
  }
  if (val == HIGH){
    if(preval == LOW){
      if (counthigh <=1){
        now = nh.now();
        now_pre = now.toSec();
      }
     counthigh++;
     if(counthigh >= 50){
      now = nh.now();
      now_post = now.toSec(); 
      msg.data = true;
      msg_time.data = now_post - now_pre;
      pub.publish(&msg);
      pub_time.publish(&msg_time);

      counthigh = 0;
      preval = HIGH;
      }
    }
    
    }
  if (val == LOW){
    counthigh = 0;
    if(preval == HIGH){
      countlow++;
    }
    if(countlow >= 5){
      countlow = 0;
      preval = LOW;
      }
    }
  nh.spinOnce();
}
