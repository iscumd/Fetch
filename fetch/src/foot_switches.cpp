#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt8MultiArray.h"

#include <string>

#include <rc/button.h>
#include <rc/time.h>

#define  BUTTON_PIN_FRONT_LEFT	1,25
#define  BUTTON_PIN_FRONT_RIGHT	1,17
#define  BUTTON_PIN_BACK_LEFT	3,20
#define  BUTTON_PIN_BACK_RIGHT	3,17

bool enableLogging;
int frequency_hz = 0.5;
static std::vector<uint8_t> buttons;
static ros::Publisher pub;

int board_init(){
	if(rc_button_init(BUTTON_PIN_FRONT_LEFT, RC_BTN_POLARITY_NORM_LOW, RC_BTN_DEBOUNCE_DEFAULT_US)
		|| rc_button_init(BUTTON_PIN_FRONT_RIGHT, RC_BTN_POLARITY_NORM_LOW, RC_BTN_DEBOUNCE_DEFAULT_US)
		|| rc_button_init(BUTTON_PIN_BACK_LEFT, RC_BTN_POLARITY_NORM_LOW, RC_BTN_DEBOUNCE_DEFAULT_US)
		|| rc_button_init(BUTTON_PIN_BACK_RIGHT, RC_BTN_POLARITY_NORM_LOW, RC_BTN_DEBOUNCE_DEFAULT_US)){
			ROS_ERROR("Failed to init buttons.");
			return -1;
	}
	return 0;
}

static void publish_message(){
	std_msgs::UInt8MultiArray msg;
	msg.data = buttons;
	pub.publish(msg);
}

static void __on_front_left_press(void){
	buttons.at(0) = 1;
	publish_message();
	return;
}
static void __on_front_left_release(void){
	buttons.at(0) = 0;
	publish_message();
	return;
}

static void __on_front_right_press(void){
	buttons.at(1) = 1;
	publish_message();
	return;
}
static void __on_front_right_release(void){
	buttons.at(1) = 0;
	publish_message();
	return;
}

static void __on_back_left_press(void){
	buttons.at(2) = 1;
	publish_message();
	return;
}
static void __on_back_left_release(void){
	buttons.at(2) = 0;
	publish_message();
	return;
}

static void __on_back_right_press(void){
	buttons.at(3) = 1;
	publish_message();
	return;
}
static void __on_back_right_release(void){
	buttons.at(3) = 0;
	publish_message();
	return;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "foot_switches");

	ros::NodeHandle n;

	n.param("foot_switches_enable_logging", enableLogging, false);

	pub = n.advertise<std_msgs::UInt8MultiArray>("foot_switches", 100);
	ros::spinOnce();

	if(board_init()) return -1;

	// initilize buttons as pressed
	for(int i = 0; i < 4; i++){
		buttons.push_back(1);
	}
	
	rc_button_set_callbacks(BUTTON_PIN_FRONT_LEFT, __on_front_left_press, __on_front_left_release);
	rc_button_set_callbacks(BUTTON_PIN_FRONT_RIGHT, __on_front_right_press, __on_front_right_release);
	rc_button_set_callbacks(BUTTON_PIN_BACK_LEFT, __on_back_left_press, __on_back_left_release);
	rc_button_set_callbacks(BUTTON_PIN_BACK_RIGHT, __on_back_right_press, __on_back_right_release);

	// ros::Rate loopRate(frequency_hz);
	while(ros::ok()) {
		ros::spinOnce();
		rc_usleep(frequency_hz*1000000);
		// loopRate.sleep();
	}
	rc_button_cleanup();
	return 0;
}
