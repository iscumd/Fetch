#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"

#include <string>

ros::Publisher legPub;

bool enableLogging;

int main(int argc, char **argv){
	ros::init(argc, argv, "leg_mapping");

	ros::NodeHandle n;

	n.param("leg_mapping_enable_logging", enableLogging, false);

	legPub = n.advertise<geometry_msgs::Quaternion>("leg_mapping", 5);

	// ros::Subscriber joystickSub = n.subscribe("joystick/xinput", 5, joystickCallback);

	ros::spin();

	return 0;
}
