#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "isc_joy/xinput.h"

#include <string>

ros::Publisher servoAnglePub;

int frequency_hz = 2;
bool enableMovement;

void joystickCallback(const isc_joy::xinput::ConstPtr& joy){
	enableMovement = joy->A ? !enableMovement : enableMovement;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "test_up_and_down");

	ros::NodeHandle n;

	servoAnglePub = n.advertise<std_msgs::Float32MultiArray>("leg_mapping", 5);

	ros::Subscriber joystickSub = n.subscribe("joystick/xinput", 5, joystickCallback);

    double angle = 180;
    int down = 0;
	ros::Rate loopRate(frequency_hz);
	while(ros::ok()) {
		ros::spinOnce();
        if(enableMovement){
	        std_msgs::Float32MultiArray servoAngle;
            for(int i = 0; i < 8; i++){
                servoAngle.data.push_back(angle * down);
            }
	        servoAnglePub.publish(servoAngle);
        }
        down = !down;
		loopRate.sleep();
	}

	return 0;
}
