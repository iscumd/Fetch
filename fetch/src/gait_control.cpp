#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "fetch/RhoThetaQArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include "rc/mpu.h"

#include <string>



// -------- ROS Specific ---------

ros::Publisher gaitPub;
std_msgs::UInt8MultiArray footSwitch;

// ---- Variables and Classes ----

ros::Subscriber eulerSub;
static rc_mpu_data_t data;
bool enableLogging;
float32<vector> deltaRho = 0;

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

	
	eulerSub = n.subscribe("orientation_control", 5, orientationControlCallback);
	
	ros::spin();

	return 0;
}


void orientationControlCallback(geometry_msgs::Quaternion::orientation &msg){
	//Message reads in Pitch, Roll, Yaw(msg->data.x, msg->data.y, msg->data.z respectively)
	//Pitch should be no greater than |30 degrees| (absolute value)
	//Roll should be no greater than 
	
}	