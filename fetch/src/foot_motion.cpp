#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"

#include <string>

ros::Publisher footPub;

bool enableLogging;

int main(int argc, char **argv){
	ros::init(argc, argv, "foot_motion");

	ros::NodeHandle n;

	n.param("foot_motion_enable_logging", enableLogging, false);

	footPub = n.advertise<geometry_msgs::Quaternion>("foot_motion", 5);

	// ros::Subscriber joystickSub = n.subscribe("joystick/xinput", 5, joystickCallback);

	ros::spin();

	return 0;
}
