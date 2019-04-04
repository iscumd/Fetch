#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "fetch/RhoThetaQArray.h"
#include "std_msgs/UInt8MultiArray.h"

#include <string>



// -------- ROS Specific ---------

ros::Publisher gaitPub;
std_msgs::UInt8MultiArray footSwitch;

// ---- Variables and Classes ----

bool enableLogging;

class stabMargin{
	float plus, minus;
};

// ----- Callback Functions ------

void switchCallback(const std_msgs::UInt8MultiArray::ConstPtr& switchCallback)
{
	footSwitch = *switchCallback;	// rtq input
}

// ---------- Functions ----------

stabMargin stabilityCalc(int testLeg, std_msgs::UInt8MultiArray footSw){
	
}

// ------------- Main ------------

int main(int argc, char **argv){
	ros::init(argc, argv, "gait_control");

	ros::NodeHandle n;

	n.param("gait_control_enable_logging", enableLogging, false);

	gaitPub = n.advertise<fetch::RhoThetaQArray>("gait_control", 5);

	//ros::Subscriber joystickSub = n.subscribe("joystick/xinput", 5, joystickCallback);
	ros::Subscriber switchSub = n.subscribe("foot_switches", 5, switchCallback);

	ros::spin();

	return 0;
}
