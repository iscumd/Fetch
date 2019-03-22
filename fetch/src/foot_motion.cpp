#include "ros/ros.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "foot_motion");

	ros::NodeHandle n;

	ROS_WARN("The Robot Control Library is only installed on the BeagleBone, so this node will remain empty in this branch so that we can build and test on our own machines without that library. This node will now exit.");

	return 0;
}
