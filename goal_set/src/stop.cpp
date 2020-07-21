#include "ros/ros.h"
#include "std_msgs/Bool.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "stop");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::Bool>("shutdown", 1000);
	ros::Rate loop_rate(10);
  	while (ros::ok()){
    		std_msgs::Bool msg;
		msg.data = false;
    		chatter_pub.publish(msg);
    		ros::spinOnce();
    		loop_rate.sleep();
	}
  	return 0;
}
