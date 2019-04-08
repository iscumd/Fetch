#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Twist.h"
#include "fetch/RhoThetaQArray.h"
#include "std_msgs/UInt8MultiArray.h"
//#include "rc/mpu.h"

#include <string>

#define FREQ 20

// -------- ROS Specific ---------

// ---- Variables and Classes ----

bool enableLogging;
double leftRollThresh, rightRollThresh, forwardPitchThresh;
double backwardPitchThresh;

class stabMargin{
public:
	float plus, minus;
	float min(void){
		if(abs(plus) < abs(minus)) return abs(plus);
		else return abs(minus); 
	}
};

class robot{
public:
	std_msgs::UInt8MultiArray footSwitch;
	geometry_msgs::Polygon footPosition;
	geometry_msgs::Quaternion orientation;
	geometry_msgs::Twist velocity;
	
	fetch::RhoThetaQArray rtq;
	float deltaRho[4];
	int state[4];

	stabMargin stability(int testLeg){
		// input what leg we are considering raising
		// if you want just the current state, set testLeg to -1 in the function call

		std_msgs::UInt8MultiArray localFootSw = footSwitch;
		stabMargin stab;
		float sHolder = 0;
		stab.plus = 0;
		stab.minus = 0;

		if(testLeg != -1) localFootSw.data[testLeg] = 0;

		for(int i=0;i<3;i++){
			if(localFootSw.data[i] != 0 && localFootSw.data[i+1] != 0){
				sHolder = (footPosition.points[i].x + footPosition.points[i+1].x) / 2;
				if(sHolder > stab.plus) stab.plus = sHolder;
				else if(sHolder < stab.minus) stab.minus = sHolder;
			}
		}
		return stab;
	};
};

robot brandon;

// ----- Callback Functions ------

void manualControlCallback(const geometry_msgs::Twist::ConstPtr& msg){
	// input velocity
	// add format of multiarray for ease-of-use
	brandon.velocity = *msg;	// velocity
}

void switchCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg){
	// current foot switch states
	// add format of multiarray for ease-of-use
	brandon.footSwitch = *msg;	// switch state input
}

void footCallback(const geometry_msgs::Polygon::ConstPtr& msg){
	// current foot positions
	// brandon.footPosition.points[i].x
	// brandon.footPosition.points[i].y
	// brandon.footPosition.points[i].z
	brandon.footPosition = *msg;
}

void orientationControlCallback(const geometry_msgs::Quaternion::ConstPtr& msg){
	
	
	//Message reads in Pitch, Roll, Yaw(msg->data.x, msg->data.y, msg->data.z respectively)
	//Pitch should be no greater than |30 degrees| (absolute value)
	//Roll should be no greater than 
	brandon.orientation = *msg;
	
	//Units in cm for deltaRho
	//Will adjust the deltaRho offsets for each leg based on the offset data to adjust balance
	if(brandon.orientation.x > forwardPitchThresh){
		brandon.deltaRho[0]--;
		brandon.deltaRho[1]--;
		brandon.deltaRho[2]++;
		brandon.deltaRho[3]++;
	}

	if(brandon.orientation.x < backwardPitchThresh){
		brandon.deltaRho[0]++;
		brandon.deltaRho[1]++;
		brandon.deltaRho[2]--;
		brandon.deltaRho[3]--;
	}

	if(brandon.orientation.y < leftRollThresh){
		brandon.deltaRho[0]--;
		brandon.deltaRho[2]--;
		brandon.deltaRho[1]++;
		brandon.deltaRho[3]++;
	}

	if(brandon.orientation.y > rightRollThresh){
		brandon.deltaRho[0]++;
		brandon.deltaRho[2]++;
		brandon.deltaRho[1]--;
		brandon.deltaRho[3]--;
	}
}	

// ---------- Functions ----------
void lift(int leg){
	brandon.rtq.rho[leg] -= 5;
	brandon.deltaRho[leg] -=5;
}

void swing(int leg){

}

void drop(int leg){
	if (brandon.footSwitch.data[leg] == false){
		brandon.rtq.rho[leg] += 1;
		brandon.deltaRho[leg] +=1;
	}else {
		brandon.state[leg] = 3;
		//TODO add timestamps per leg?
	}

}

void stride(int leg){
	if (brandon.footSwitch.data[leg] == false){
		brandon.rtq.rho[leg] += 1;
		brandon.deltaRho[leg] +=1;
	}
	brandon.rtq.q[leg] -= brandon.velocity.linear.x/FREQ;
}

// ------------- Main ------------

int main(int argc, char **argv){
	
	// define name of node and start
	ros::init(argc, argv, "gait_control");
    
	// The first nodehandle constructed will fully initialize this node
	ros::NodeHandle n;
    
	// specify loop frequency, works with Rate::sleep to sleep for the correct time
    ros::Rate loop_rate(FREQ);

	// get ros params
	n.param("gait_control_enable_logging", enableLogging, false);
   
    //Pitch and Roll Thresholds.
	n.param("left_roll_threshold", leftRollThresh, -30.0);
	n.param("right_roll_threshold", rightRollThresh, 30.0);
	n.param("forward_pitch_threshold", forwardPitchThresh, 30.0);
	n.param("backward_pitch_threshold", backwardPitchThresh, -30.0);
	

	// define topic name to publish to and queue size
	ros::Publisher gaitPub = n.advertise<fetch::RhoThetaQArray>("gait_control", 5);
    
	// define topic names to subscribe to and queue size
	ros::Subscriber switchSub = n.subscribe("foot_switches", 5, switchCallback);
	ros::Subscriber footSub = n.subscribe("foot_position", 5, footCallback);
	ros::Subscriber eulerSub = n.subscribe("orientation_control", 5, orientationControlCallback);
	ros::Subscriber manualControlSub = n.subscribe("manual_control", 5, manualControlCallback);

	while(ros::ok()){
		ros::spinOnce();

		// decision making and state assignment goes here

		for (int i=0;i<4;i++){
			switch(brandon.state[i]){
				//* 0 for swing function 1 for stride function

			case 0:
				lift(i);

			case 1:
				swing(i);

			case 2:
				drop(i);
			
			case 3:
				stride(i);

			};
		};

		gaitPub.publish(brandon.rtq);

		if(enableLogging) ROS_INFO("Execution time: [%f]", loop_rate.cycleTime().toSec());
        
		// The thing is, Bob, it's not that I'm lazy, it's that I just don't care.
		loop_rate.sleep();
	}

	return 0;
}

