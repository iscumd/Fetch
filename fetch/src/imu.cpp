#include "ros/ros.h"
#include "fetch/OrientationRPY.h"

// #include <rc/led.h>
#include <rc/mpu.h>

#include <string>

// bus for Robotics Cape and BeagleboneBlue is 2
#define I2C_BUS 2

rc_mpu_data_t data; //Data from MPU
static ros::Publisher orientationPub;
int publish_rate_hz;

static void mpu_dmp_callback(void){
	fetch::OrientationRPY msg;
	msg.roll = data.dmp_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG;
	msg.pitch = data.dmp_TaitBryan[TB_PITCH_X]*RAD_TO_DEG;
	msg.yaw = data.dmp_TaitBryan[TB_YAW_Z]*RAD_TO_DEG;
	orientationPub.publish(msg);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "orientation_control");

	ros::NodeHandle n;

	// n.param("orientation_control_enable_logging", enableLogging, false);
	n.param("orientation_control_publish_rate", publish_rate_hz, 10);
	if(publish_rate_hz>200 || publish_rate_hz<4){
		ROS_ERROR("orientation_control_publish_rate must be between 4 & 200");
		return -1;
	}

	orientationPub = n.advertise<fetch::OrientationRPY>("orientation_control", 5);
	
	rc_mpu_config_t mpu_config = rc_mpu_default_config();
    mpu_config.dmp_sample_rate = publish_rate_hz;
    mpu_config.orient = ORIENTATION_Y_UP;
	mpu_config.i2c_bus = I2C_BUS;
	
	//Initialize MPU
	if(rc_mpu_initialize_dmp(&data, mpu_config)){
		ROS_ERROR("Failed to initialize IMU");
		// rc_led_blink(RC_LED_RED, 5, 5);
		// rc_led_cleanup();
		return -1;
	}

	rc_mpu_set_dmp_callback(&mpu_dmp_callback);
	
	ros::spin();
	
	rc_mpu_power_off();
	return 0;
}



