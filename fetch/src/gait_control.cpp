#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"

#include <string>

ros::Publisher gaitPub;

bool enableLogging;

int main(int argc, char **argv){
	ros::init(argc, argv, "gait_control");

	ros::NodeHandle n;

	n.param("gait_control_enable_logging", enableLogging, false);

	gaitPub = n.advertise<geometry_msgs::Quaternion>("gait_control", 5);

	// ros::Subscriber joystickSub = n.subscribe("joystick/xinput", 5, joystickCallback);

	ros::spin();

	return 0;
}
