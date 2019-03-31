#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

#include <string>

#include <rc/adc.h>
#include <rc/servo.h>
#include <rc/time.h>

bool enableLogging;
bool useOnboardPower;
int frequency_hz; //frequency to send pulses
float default_pulse_width = 1.5;
int number_of_channels = 8;
std::vector<float> pulse_width_ms;

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

//channel 0 = all channels
int move_servo(int channel, float pulse_width_ms){
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

void pulseWidthCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	//https://gist.github.com/alexsleat/1372845/7e39518cfa12ac91aca4378843e55862eb9ed41d
	int i = 0;
	for(std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
	{
		if (i >= number_of_channels) {
			ROS_ERROR("More servo motor angles sent than channels available. Truncating message read at %i channels.", number_of_channels);
			return;
		}
		pulse_width_ms.push_back(*it);
		i++;
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "foot_servos");

	ros::NodeHandle n;

	n.param("foot_servos_enable_logging", enableLogging, false);
	n.param("foot_servos_use_onboard_power", useOnboardPower, false);
	n.param("foot_servos_pulse_frequency", frequency_hz, 50);

	for(int i = 0; i < number_of_channels; i++){
		pulse_width_ms.push_back(default_pulse_width);
	}
	ros::Subscriber pulseWidthSub = n.subscribe("foot/pulse_width_ms", 5, pulseWidthCallback);
	ros::spinOnce();

	if(board_init()) return -1;

	ros::Rate loopRate(frequency_hz); //Hz
	while(ros::ok()) {
		ros::spinOnce();
		
		for(int i = 0; i < pulse_width_ms.size(); i++){
			move_servo(i+1, pulse_width_ms.at(i));
		}

		loopRate.sleep();
	}
	rc_usleep(50000);
	// turn off power rail and cleanup
	rc_servo_power_rail_en(0);
	rc_servo_cleanup();
	return 0;
}
