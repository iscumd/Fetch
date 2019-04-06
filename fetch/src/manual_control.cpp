#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "isc_joy/xinput.h"

#include <string>

ros::Publisher manualPub;

bool flipForwardBackward = false;
bool flipLeftRight = false;

double speedMultiplier;
double turnMultiplier;

bool enableLogging;

void joystickCallback(const isc_joy::xinput::ConstPtr& joy){	
	/* This fires every time a button is pressed/released
	and when an axis changes (even if it doesn't leave the
	deadzone). */

	bool enableDriving = joy->LB; //the dead man's switch

	double joySpeed = 0.0, joyRoll = 0.0, joyPitch = 0.0, joyTurn = 0.0;
	joySpeed = joy->LeftStick_UD;
	joyTurn = joy->LeftStick_LR;
	joyRoll = joy->RightStick_LR;
	joyPitch = joy->RightStick_UD;

	geometry_msgs::Twist msg;
	msg.linear.x = enableDriving ? joySpeed : 0.0;
	msg.angular.x = enableDriving ? joyRoll : 0.0;
	msg.angular.y = enableDriving ? joyPitch : 0.0;
	msg.angular.z = enableDriving ? joyTurn : 0.0;
	manualPub.publish(msg);

	if(enableLogging) ROS_INFO("Manual Control: %s linear.x=%f angular.x=%f angular.y=%f angular.z=%f", 
        joy->LB ? "on" : "off", msg.linear.x, msg.angular.x, msg.angular.y, msg.angular.z);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "manual_control");

	ros::NodeHandle n;

	n.param("manual_control_enable_logging", enableLogging, false);

	manualPub = n.advertise<geometry_msgs::Twist>("manual_control", 5);

	ros::Subscriber joystickSub = n.subscribe("joystick/xinput", 5, joystickCallback);

	ros::spin();

	return 0;
}
