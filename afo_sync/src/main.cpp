#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "afo_sync");
	ros::NodeHandle n;
    ros::Rate loop_rate(100);
    ros::Publisher afo_sync_pub = n.advertise<std_msgs::Bool>("/afo_sync/sync", 100);

    
    std_msgs::Bool msg_sync;

    bool current_sync = true;
    msg_sync.data = current_sync;
    afo_sync_pub.publish(msg_sync);
char _tmp;
    while(ros::ok()){
	cout << "Toggle Sync signal: ";
	cin >> _tmp;
	msg_sync.data = current_sync;
	current_sync = !current_sync;
	afo_sync_pub.publish(msg_sync);
cout << "afo_sync Node - msg 'sync' published, value = " << !current_sync << endl;
	}
	cout << "afo_sync Node - ros end - main end" << endl;

    return 1;
}
