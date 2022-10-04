#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <thread>
#include <limits>

using namespace std;

bool endSign = false;
int toggle(bool* current_sync, bool* current_zero){
	int _tmp;
	while(!endSign){
		cout << "Toggle Sync signal: ";
		scanf("%d", &_tmp);
		if (_tmp == 1){
			*current_zero = !(*current_zero);
			cout << "afo_sync Node - msg 'zero' published, value = " << *current_zero << endl;
		}
		else{
			*current_sync = !(*current_sync);
			cout << "afo_sync Node - msg 'sync' published, value = " << *current_sync << endl;
		}
	}
	return 1;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "afo_sync");
	ros::NodeHandle n;
	int rr;
    	ros::Publisher afo_sync_pub = n.advertise<std_msgs::Bool>("/afo_sync/sync", 100);
    	ros::Publisher afo_zero_pub = n.advertise<std_msgs::Bool>("/afo_sync/zero", 100);

	std_msgs::Bool msg_sync;
    	std_msgs::Bool msg_zero;

    	bool* current_sync = new bool;
    	bool* current_zero = new bool;
    	*current_sync = false;
    	*current_zero = false;

	int _tmp;
    while(ros::ok()){
cout << "Toggle Sync signal: ";
cin >> _tmp;		
if (_tmp == 1){
			*current_zero = !(*current_zero);
			cout << "afo_sync Node - msg 'zero' published, value = " << *current_zero << endl;
		}
		else{
			*current_sync = !(*current_sync);
			cout << "afo_sync Node - msg 'sync' published, value = " << *current_sync << endl;
		}
	msg_zero.data = *current_zero;
	msg_sync.data = *current_sync;
	afo_zero_pub.publish(msg_zero);
	afo_sync_pub.publish(msg_sync);
	}
	cout << "afo_sync Node - ros end - main end" << endl;
endSign = true;
    return 1;
}
