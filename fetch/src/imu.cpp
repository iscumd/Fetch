#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "rc/mpu.h"

#include <string>

static rc_mpu_data_t data;
ros::Publisher orientationPub;
geometry_msgs::orienation msg;


int main(int argc, char **argv){
	ros::init(argc, argv, "orientation_control");

	ros::NodeHandle n;

	n.param("orientation_control_enable_logging", enableLogging, false);
	orientationPub = n.advertise<geometry_msgs::Quaternion>("orientation_control", 5);
	
	//Returns the euler angles in degrees
	msg->data.x = dmp_TaitBryan[TB_PITCH_X]*RAD_TO_DEG; //Pitch
	msg->data.y = dmp_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG;//Roll
	msg->data.z = dmp_TaitBryan[TB_YAW_Z]*RAD_TO_DEG; //Yaw
	
	//Pitch = data.x 	Roll = data.y   	Yaw = data.z
	orientationPub.publish(msg);
	ros::spin();

	return 0;
}



