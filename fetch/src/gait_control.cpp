#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Polygon.h"
#include "fetch/RhoThetaQArray.h"
#include "std_msgs/UInt8MultiArray.h"
//#include "rc/mpu.h"

#include <string>



// -------- ROS Specific ---------

ros::Publisher gaitPub;
ros::Subscriber eulerSub;

std_msgs::UInt8MultiArray footSwitch;
geometry_msgs::Polygon footPos;
geometry_msgs::Quaternion orient;

// ---- Variables and Classes ----

// static rc_mpu_data_t data;
// float32<vector> deltaRho = 0; 

bool enableLogging;

class stabMargin{
	float plus, minus;
public:
	float min(void){
		if(abs(plus) > abs(minus)) {
			return abs(plus);
		}else{ return abs(minus); }
	}
};

// ----- Callback Functions ------

void switchCallback(const std_msgs::UInt8MultiArray::ConstPtr& switchCallback)
{
	footSwitch = *switchCallback;	// switch state input
}

void footCallback(const geometry_msgs::Polygon::ConstPtr& footCallback){
	footPos = *footCallback;		// current foot positions
}

void orientationControlCallback(const geometry_msgs::Quaternion::ConstPtr& orientCallback){
	//Message reads in Pitch, Roll, Yaw(msg->data.x, msg->data.y, msg->data.z respectively)
	//Pitch should be no greater than |30 degrees| (absolute value)
	//Roll should be no greater than 
	orient = *orientCallback;
	
}	

// ---------- Functions ----------

stabMargin stabilityCalc(geometry_msgs::Polygon footPositions, std_msgs::UInt8MultiArray footSw, int testLeg){
	
}

// ------------- Main ------------

int main(int argc, char **argv){
	ros::init(argc, argv, "gait_control");

	ros::NodeHandle n;

	n.param("gait_control_enable_logging", enableLogging, false);

	gaitPub = n.advertise<fetch::RhoThetaQArray>("gait_control", 5);

	//ros::Subscriber joystickSub = n.subscribe("joystick/xinput", 5, joystickCallback);
	ros::Subscriber switchSub = n.subscribe("foot_switches", 5, switchCallback);
	ros::Subscriber footSub = n.subscribe("foot_position", 5, footCallback);
	
	eulerSub = n.subscribe("orientation_control", 5, orientationControlCallback);
	
	ros::spin();

	return 0;
}

