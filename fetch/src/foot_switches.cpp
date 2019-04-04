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
std::vector<uint8_t> buttons;
static std::vector<uint8_t> buttons_volitile;
ros::Publisher pub;

int board_init(){
	if(rc_button_init(BUTTON_PIN_FRONT_LEFT, RC_BTN_POLARITY_NORM_LOW, RC_BTN_DEBOUNCE_DEFAULT_US)
		|| rc_button_init(BUTTON_PIN_FRONT_RIGHT, RC_BTN_POLARITY_NORM_LOW, RC_BTN_DEBOUNCE_DEFAULT_US)
		|| rc_button_init(BUTTON_PIN_BACK_LEFT, RC_BTN_POLARITY_NORM_LOW, RC_BTN_DEBOUNCE_DEFAULT_US)
		|| rc_button_init(BUTTON_PIN_BACK_RIGHT, RC_BTN_POLARITY_NORM_LOW, RC_BTN_DEBOUNCE_DEFAULT_US)){
			ROS_ERROR("Failed to initialize buttons.");
			return -1;
	}
	return 0;
}

void publish_message(){
	buttons = buttons_volitile;
	std_msgs::UInt8MultiArray msg;
	msg.data = buttons;
	pub.publish(msg);
}

static void __on_front_left_press(void){
	buttons_volitile.at(0) = 1;
	return;
}
static void __on_front_left_release(void){
	buttons_volitile.at(0) = 0;
	return;
}

static void __on_front_right_press(void){
	buttons_volitile.at(1) = 1;
	return;
}
static void __on_front_right_release(void){
	buttons_volitile.at(1) = 0;
	return;
}

static void __on_back_left_press(void){
	buttons_volitile.at(2) = 1;
	return;
}
static void __on_back_left_release(void){
	buttons_volitile.at(2) = 0;
	return;
}

static void __on_back_right_press(void){
	buttons_volitile.at(3) = 1;
	return;
}
static void __on_back_right_release(void){
	buttons_volitile.at(3) = 0;
	return;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "foot_switches");

	ros::NodeHandle n;

	n.param("foot_switches_enable_logging", enableLogging, false);

	pub = n.advertise<std_msgs::UInt8MultiArray>("foot_switches", 100);
	ros::spinOnce();

	if(board_init()) return -1;

	// initilize buttons' state
	buttons_volitile.push_back(rc_button_get_state(BUTTON_PIN_FRONT_LEFT));
	buttons_volitile.push_back(rc_button_get_state(BUTTON_PIN_FRONT_RIGHT));
	buttons_volitile.push_back(rc_button_get_state(BUTTON_PIN_BACK_LEFT));
	buttons_volitile.push_back(rc_button_get_state(BUTTON_PIN_BACK_RIGHT));
	publish_message();
	ros::spinOnce();
	
	rc_button_set_callbacks(BUTTON_PIN_FRONT_LEFT, __on_front_left_press, __on_front_left_release);
	rc_button_set_callbacks(BUTTON_PIN_FRONT_RIGHT, __on_front_right_press, __on_front_right_release);
	rc_button_set_callbacks(BUTTON_PIN_BACK_LEFT, __on_back_left_press, __on_back_left_release);
	rc_button_set_callbacks(BUTTON_PIN_BACK_RIGHT, __on_back_right_press, __on_back_right_release);

	// ros::Rate loopRate(frequency_hz);
	while(ros::ok()) {
		if(buttons_volitile != buttons){
			publish_message();
		}
		ros::spinOnce();
		rc_usleep(frequency_hz*1000000);
		// loopRate.sleep();
	}
	rc_button_cleanup();
	return 0;
}
