#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "rc/mpu.h"

#include <string>

ros::Publisher gaitPub;
ros::Subscriber eulerSub;
static rc_mpu_data_t data;
bool enableLogging;
float32<vector> deltaRho = 0;

int main(int argc, char **argv){
	ros::init(argc, argv, "gait_control");

	ros::NodeHandle n;

	n.param("gait_control_enable_logging", enableLogging, false);
	
	eulerSub = n.subscribe("orientation_control", 5, orientationControlCallback);
	
	ros::spin();

	return 0;
}


void orientationControlCallback(geometry_msgs::Quaternion::orientation &msg){
	//Message reads in Pitch, Roll, Yaw(msg->data.x, msg->data.y, msg->data.z respectively)
	//Pitch should be no greater than |30 degrees| (absolute value)
	//Roll should be no greater than 
	
}	