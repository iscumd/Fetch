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
volatile static uint8_t button0_volatile;
volatile static uint8_t button1_volatile;
volatile static uint8_t button2_volatile;
volatile static uint8_t button3_volatile;
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
	button0_volatile = buttons.at(0);
	button1_volatile = buttons.at(1);
	button2_volatile = buttons.at(2);
	button3_volatile = buttons.at(3);
	std_msgs::UInt8MultiArray msg;
	msg.data = buttons;
	pub.publish(msg);
}

static void __on_front_left_press(void){
	button0_volatile = RC_BTN_STATE_PRESSED;
	return;
}
static void __on_front_left_release(void){
	button0_volatile = RC_BTN_STATE_RELEASED;
	return;
}

static void __on_front_right_press(void){
	button1_volatile = RC_BTN_STATE_PRESSED;
	return;
}
static void __on_front_right_release(void){
	button1_volatile = RC_BTN_STATE_RELEASED;
	return;
}

static void __on_back_left_press(void){
	button2_volatile = RC_BTN_STATE_PRESSED;
	return;
}
static void __on_back_left_release(void){
	button2_volatile = RC_BTN_STATE_RELEASED;
	return;
}

static void __on_back_right_press(void){
	button3_volatile = RC_BTN_STATE_PRESSED;
	return;
}
static void __on_back_right_release(void){
	button3_volatile = RC_BTN_STATE_RELEASED;
	return;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "foot_switches");

	ros::NodeHandle n;

	n.param("foot_switches_enable_logging", enableLogging, false);

	pub = n.advertise<std_msgs::UInt8MultiArray>("foot_switches", 100);
	ros::spinOnce();

	if(board_init()) return -1;

	// initialize buttons array
	for(int i = 0; i < 4; i++){
		buttons.push_back(1);
	}
	// initialize buttons' state
	button0_volatile = rc_button_get_state(BUTTON_PIN_FRONT_LEFT);
	button1_volatile = rc_button_get_state(BUTTON_PIN_FRONT_RIGHT);
	button2_volatile = rc_button_get_state(BUTTON_PIN_BACK_LEFT);
	button3_volatile = rc_button_get_state(BUTTON_PIN_BACK_RIGHT);
	publish_message();
	ros::spinOnce();
	
	rc_button_set_callbacks(BUTTON_PIN_FRONT_LEFT, __on_front_left_press, __on_front_left_release);
	rc_button_set_callbacks(BUTTON_PIN_FRONT_RIGHT, __on_front_right_press, __on_front_right_release);
	rc_button_set_callbacks(BUTTON_PIN_BACK_LEFT, __on_back_left_press, __on_back_left_release);
	rc_button_set_callbacks(BUTTON_PIN_BACK_RIGHT, __on_back_right_press, __on_back_right_release);

	// ros::Rate loopRate(frequency_hz);
	while(ros::ok()) {
		if(button0_volatile != buttons.at(0) || button1_volatile != buttons.at(1) 
			|| button2_volatile != buttons.at(2) || button3_volatile != buttons.at(3)){
			publish_message();
		}
		ros::spinOnce();
		rc_usleep(frequency_hz*1000000);
		// loopRate.sleep();
	}
	rc_button_cleanup();
	return 0;
}
