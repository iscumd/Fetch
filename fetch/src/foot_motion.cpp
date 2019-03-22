#include "ros/ros.h"

#include <string>
// #include <stdlib.h> // for atoi

#include <rc/adc.h>
#include <rc/servo.h>
#include <rc/time.h>

bool enableLogging;
bool useOnboardPower;
int frequency_hz; //frequency to send pulses

int board_init(){
	if (useOnboardPower) {
		// read adc to make sure battery is connected
        if(rc_adc_init()){
                ROS_ERROR("failed to run rc_adc_init()");
                return -1;
        }
        if(rc_adc_batt()<6.0){
                ROS_ERROR("battery disconnected or insufficiently charged to drive servos");
                return -1;
        }
	}

	// initialize PRU
	if(rc_servo_init()) return -1;

	if (useOnboardPower) {
		// turn on power
		ROS_INFO("Turning On 6V Servo Power Rail");
		rc_servo_power_rail_en(1);
	}
	return 0;
}

//0 = all channels
int move_servo(int channel, float width_ms){
	int width_us = width_ms * 1000;
	if(width_us<10){
			ROS_ERROR("Width in microseconds must be >10");
			return -1;
	}
	if(rc_servo_send_pulse_us(channel, width_us)==-1) return -1;
	return 0;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "foot_motion");

	ros::NodeHandle n;

	n.param("foot_motion_enable_logging", enableLogging, false);
	n.param("foot_motion_use_onboard_power", useOnboardPower, false);
	n.param("foot_motion_pulse_frequency", frequency_hz, 50);

	// ros::Subscriber joystickSub = n.subscribe("joystick/xinput", 5, joystickCallback);

	if(board_init()) return -1;

	ros::Rate loopRate(frequency_hz); //Hz
	while(ros::ok()) {
		ros::spinOnce();
		
		move_servo(0, 0.9);

		loopRate.sleep();
		
		move_servo(0, 1.5);

		loopRate.sleep();
		
		move_servo(0, 2.1);

		loopRate.sleep();
		
		move_servo(0, 1.5);

		loopRate.sleep();
	}
	rc_usleep(50000);
	// turn off power rail and cleanup
	rc_servo_power_rail_en(0);
	rc_servo_cleanup();
	return 0;
}
