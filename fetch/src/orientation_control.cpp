#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"

#include <string>

ros::Publisher orientationPub;

bool enableLogging;

int main(int argc, char **argv){
	ros::init(argc, argv, "orientation_control");

	ros::NodeHandle n;

	n.param("orientation_control_enable_logging", enableLogging, false);

	orientationPub = n.advertise<geometry_msgs::Quaternion>("orientation_control", 5);

	// ros::Subscriber joystickSub = n.subscribe("joystick/xinput", 5, joystickCallback);

	ros::spin();

	return 0;
}
