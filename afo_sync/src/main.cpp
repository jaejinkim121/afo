#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "afo_sync");
	ros::NodeHandle n;
    ros::Rate loop_rate(100);
    ros::Publisher afo_sync_pub = n.advertise<std_msgs::Bool>("/afo_sync/sync", 100);
    ros::Publisher afo_zero_pub = n.advertise<std_msgs::Bool>("/afo_sync/zero", 100);

    std_msgs::Bool msg_sync;
    std_msgs::Bool msg_zero;

    bool current_sync = false;
    bool current_zero = false;
    msg_sync.data = current_sync;
    afo_sync_pub.publish(msg_sync);
char _tmp;
    while(ros::ok()){
	cout << "Toggle Sync signal: ";
	cin >> _tmp;
        if (_tmp == '0'){
		current_zero = !current_zero;
		msg_zero.data = current_zero;
		afo_zero_pub.publish(msg_zero);
		cout << "afo_sync Node - msg 'zero' published, value = " << current_zero <<endl;
	}
	else{
		current_sync = !current_sync;
		msg_sync.data = current_sync;
		afo_sync_pub.publish(msg_sync);
		cout << "afo_sync Node - msg 'sync' published, value = " << !current_sync << endl;
		}
	}
	cout << "afo_sync Node - ros end - main end" << endl;

    return 1;
}
