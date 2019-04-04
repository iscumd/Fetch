#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "rc/mpu.h"

#include <string>

static rc_mpu_data_t data; //Data from MPU
ros::Publisher orientationPub;
geometry_msgs::orienation msg;


int main(int argc, char **argv){
	ros::init(argc, argv, "orientation_control");

	ros::NodeHandle n;

	n.param("orientation_control_enable_logging", enableLogging, false);
	orientationPub = n.advertise<geometry_msgs::Quaternion>("orientation_control", 5);
	
	rc_mpu_config_t mpu_config = rc_mpu_default_config();
   
   /* Gonna see if we don't even need this
   mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
    mpu_config.orient = ORIENTATION_Y_UP;
	
	//Initialize MPU
	if(rc_mpu_initialize_dmp(&data, mpu_config)){
                fprintf(stderr,"ERROR: can't talk to IMU, all hope is lost\n");
                rc_led_blink(RC_LED_RED, 5, 5);
                return -1;
        }
	
	*/
	
	mpu_read_gyro(&data);
	//Returns the Euler angles in degrees
	msg->data.x = data.dmp_TaitBryan[TB_PITCH_X]*RAD_TO_DEG; //Pitch
	msg->data.y = data.dmp_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG;//Roll
	msg->data.z = data.dmp_TaitBryan[TB_YAW_Z]*RAD_TO_DEG; //Yaw
	
	//Pitch = data.x 	Roll = data.y   	Yaw = data.z
	orientationPub.publish(msg);
	ros::spin();

	return 0;
}



