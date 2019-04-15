#include "ros/ros.h"
#include "fetch/RhoThetaQArray.h"
#include "isc_joy/xinput.h"

#include <string>

ros::Publisher gaitControlPub;

int frequency_hz = 2;
bool enableMovement;

void joystickCallback(const isc_joy::xinput::ConstPtr& joy){
	enableMovement = joy->A ? !enableMovement : enableMovement;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "test_up_and_down");

	ros::NodeHandle n;

	gaitControlPub = n.advertise<fetch::RhoThetaQArray>("gait_control", 5);

	ros::Subscriber joystickSub = n.subscribe("joystick/xinput", 5, joystickCallback);

    double min = 12;
    double max = 30;
    bool down = true;
	ros::Rate loopRate(frequency_hz);
	while(ros::ok()) {
		ros::spinOnce();
        if(enableMovement){
	        fetch::RhoThetaQArray foot_position;
            for(int i = 0; i < 4; i++){
				if (down) {
                	foot_position.rho.push_back(min);
				}
				else{
                	foot_position.rho.push_back(max);
				}
                foot_position.theta.push_back(0);
                foot_position.q.push_back(0);
            }
	        gaitControlPub.publish(foot_position);
        }
        down = !down;
		loopRate.sleep();
	}

	return 0;
}
