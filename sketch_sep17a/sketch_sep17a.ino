 #define USE_USBCON
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <Thread.h>
#include <ThreadController.h>

ros::NodeHandle nh;
bool flagSwitch = true;
int pinIn = A0;
int valA = 0;
int valArray[21] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
std_msgs::Int16MultiArray msg_val;
ros::Publisher pub_val("/afo_arduino/analog_val", &msg_val);


bool flagSwitch = true;

volatile int lastEncoded = 0;
volatile long encoderValue = 0;

int encoderPin1 = 2;
int encoderPin2 = 3;

int lastencoderValue = 0;

int lastMSB = 0;
int lastLSB = 0;

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


void updateEncoder(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

  lastEncoded = encoded; //store this value for next time
  valArray[20] = -encoderValue * 360 / 2048 + 90;
}


void setup(){
  pinMode(pinIn, INPUT);
  nh.initNode();
  nh.advertise(pub_val);
  
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);

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
