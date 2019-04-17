#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"

#include <string>

#include <rc/adc.h>
#include <rc/servo.h>
#include <rc/time.h>

bool enableLogging;
bool useOnboardPower;
bool center_on_startup;
int frequency_hz; //frequency to send pulses
int number_of_channels = 8;
std::vector<float> servo_angles (number_of_channels, 0.0);
bool initial_position_set = false;
double lower_pulse_width_ms, upper_pulse_width_ms, lower_angle, upper_angle;

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

void center_servos(){
	float midpoint_angle = (upper_angle+lower_angle)/2;
	for(int i = 0; i < number_of_channels; i++){
		servo_angles.at(midpoint_angle);
	}
}

double map(double in, double inLower, double inUpper, double outLower, double outUpper){
	// https://stackoverflow.com/a/345204
	return (in-inLower)/(inUpper-inLower) * (outUpper-outLower) + outLower;
}

//channel 0 = all channels
int move_servo(int channel, float pulse_width_ms){
	if (pulse_width_ms > upper_pulse_width_ms) {
		pulse_width_ms = upper_pulse_width_ms;
	}
	else if (pulse_width_ms < lower_pulse_width_ms) {
		pulse_width_ms = lower_pulse_width_ms;
	}
	
	int pulse_width_us = pulse_width_ms * 1000;
	if(pulse_width_us == 0){
		return 0; //intentionally don't send anything
	}
	else if(pulse_width_us < 10){
			ROS_ERROR("Width in microseconds must be >10");
			return -1;
	}
	if(rc_servo_send_pulse_us(channel, pulse_width_us)==-1) return -1;
	return 0;
}

void servoAnglesCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	//https://gist.github.com/alexsleat/1372845/7e39518cfa12ac91aca4378843e55862eb9ed41d
	int i = 0;
	for(std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
	{
		if (i >= number_of_channels) {
			ROS_ERROR("More servo motor angles sent than channels available. Truncating message read at %i channels.", number_of_channels);
			return;
		}
		servo_angles.at(i) = *it;
		i++;
	}
	initial_position_set = true;
}

void recenterCallback(const std_msgs::Bool::ConstPtr& msg){
	if (msg->data == true) {
		center_servos();
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "foot_servos");

	ros::NodeHandle n;

	n.param("foot_servos_enable_logging", enableLogging, false);
	n.param("foot_servos_use_onboard_power", useOnboardPower, false);
	n.param("foot_servos_center_on_startup", center_on_startup, false);
	n.param("foot_servos_pulse_frequency", frequency_hz, 50);
	n.param("foot_servos_lower_pulse_width_ms", lower_pulse_width_ms, 0.9);
	n.param("foot_servos_upper_pulse_width_ms", upper_pulse_width_ms, 2.1);
	n.param("foot_servos_lower_angle", lower_angle, -90.0);
	n.param("foot_servos_upper_angle", upper_angle, 90.0);

	if (center_on_startup) {
		center_servos();
		initial_position_set = true;
	}
	ros::Subscriber servoAnglesSub = n.subscribe("leg_mapping", 5, servoAnglesCallback);
	ros::Subscriber servoRecenterSub = n.subscribe("foot_servos_recenter", 5, recenterCallback);
	ros::spinOnce();

	if(board_init()) return -1;

	ros::Rate loopRate(frequency_hz); //Hz
	while(ros::ok()) {
		ros::spinOnce();
		
		if (initial_position_set) {
			for(int i = 0; i < servo_angles.size(); i++){
				move_servo(i+1, map(servo_angles.at(i),lower_angle, upper_angle, lower_pulse_width_ms, upper_pulse_width_ms));
			}
		}

		loopRate.sleep();
	}
	rc_usleep(50000);
	// turn off power rail and cleanup
	rc_servo_power_rail_en(0);
	rc_servo_cleanup();
	return 0;
}
