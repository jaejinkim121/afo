#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <thread>
#include <limits>
#include "../include/linux_kbhit.h"

using namespace std;

// bool endSign = false;
// int toggle(bool* current_sync, bool* current_zero){
// 	int _tmp;
// 	while(!endSign){
// 		cout << "Toggle Sync signal: ";
// 		scanf("%d", &_tmp);
// 		if (_tmp == 1){
// 			*current_zero = !(*current_zero);
// 			cout << "afo_sync Node - msg 'zero' published, value = " << *current_zero << endl;
// 		}
// 		else{
// 			*current_sync = !(*current_sync);
// 			cout << "afo_sync Node - msg 'sync' published, value = " << *current_sync << endl;
// 		}
// 	}
// 	return 1;
// }

int main(int argc, char** argv){
    ros::init(argc, argv, "afo_sync");
	ros::NodeHandle n;
	int rr;

	ros::Publisher afo_sync_pub = n.advertise<std_msgs::Bool>("/afo_sync/sync", 100);
	ros::Publisher afo_zero_pub = n.advertise<std_msgs::Bool>("/afo_sync/zero", 100);
	ros::Publisher afo_command_threshold_pub = n.advertise<std_msgs::Bool>("/afo_sync/command_threshold", 100);

	std_msgs::Bool msg_sync;
    std_msgs::Bool msg_zero;
	std_msgs::Bool msg_command_threshold;

	bool* current_sync = new bool;
	bool* current_zero = new bool;
	*current_sync = false;
	*current_zero = false;

	int _tmp;
    while(ros::ok()){
		int kh = linux_kbhit();
		if(kh != -1){
			switch((char)kh){
				case '1':
					*current_zero = !(*current_zero);
					msg_zero.data = *current_zero;
					afo_zero_pub.publish(msg_zero);
					cout << "afo_sync Node - msg 'zero' published, value = " << *current_zero << endl;
					break;
				case '2':
					*current_sync = !(*current_sync);
					msg_sync.data = *current_sync;
					afo_sync_pub.publish(msg_sync);
					cout << "afo_sync Node - msg 'sync' published, value = " << *current_sync << endl;
					break;
				case 't':
					msg_command_threshold.data = true;
					afo_command_threshold_pub.publish(msg_command_threshold);
					cout << "afo_sync Node - msg 'threshold' published" << endl;
					break;
				case 'e':
					ros::shutdown();
					cout << "ROS system terminated by user key INPUT" << endl;
					break;

			}
		}
	}
	cout << "afo_sync Node - ros end - main end" << endl;
    return 1;
}
