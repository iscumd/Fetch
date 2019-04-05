#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Polygon.h"
#include "fetch/RhoThetaQArray.h"
#include "std_msgs/UInt8MultiArray.h"
//#include "rc/mpu.h"

#include <string>



// -------- ROS Specific ---------

//fetch::RhoThetaQArray rtq;
//std_msgs::UInt8MultiArray footSwitch;
//geometry_msgs::Polygon footPos;
//geometry_msgs::Quaternion orient;

// ---- Variables and Classes ----

// static rc_mpu_data_t data;
// float32<vector> deltaRho = 0;

bool enableLogging;

class stabMargin{
public:
	float plus, minus;
	float min(void){
		if(abs(plus) > abs(minus)) {
			return abs(plus);
		}else{ return abs(minus); }
	}
};

class robot{
public:
	std_msgs::UInt8MultiArray footSwitch;
	geometry_msgs::Polygon footPosition;
	geometry_msgs::Quaternion orientation;
	
	fetch::RhoThetaQArray rtq;
	float deltaRho[];
	
	stabMargin stability(){return stabilityCalc(footPosition, footSwitch, -1);};
	
	static stabMargin stabilityCalc(geometry_msgs::Polygon footPositions, std_msgs::UInt8MultiArray footSw, int testLeg){
		// input foot positions and foot switch states
		// as well as what leg we are considering raising
		//
		// if you want just the current state, set testLeg to -1 in the function call

		stabMargin S;
		float sHolder = 0;
		S.plus = 0;
		S.minus = 0;

		if(testLeg != -1){
			footSw.data[testLeg] = 0;
		}

		for(int i=0;i<3;i++){
			if(footSw.data[i] != 0 && footSw.data[i+1] != 0){
				sHolder = (footPositions.points[i].x + footPositions.points[i+1].x) / 2;
				if(sHolder > S.plus){
					S.plus = sHolder;
				}else if(sHolder < S.minus){
					S.minus = sHolder;
				}
			}
		}

		return S;
	};
};

robot brandon;

// ----- Callback Functions ------

void switchCallback(const std_msgs::UInt8MultiArray::ConstPtr& switchCallback){
	// current foot switch states
	// add format of multiarray for ease-of-use
	brandon.footSwitch = *switchCallback;	// switch state input
}

void footCallback(const geometry_msgs::Polygon::ConstPtr& footCallback){
	// current foot positions
	// brandon.footPosition.points[i].x
	// brandon.footPosition.points[i].y
	// brandon.footPosition.points[i].z
	brandon.footPosition = *footCallback;
}

void orientationControlCallback(const geometry_msgs::Quaternion::ConstPtr& orientCallback){
	//Message reads in Pitch, Roll, Yaw(msg->data.x, msg->data.y, msg->data.z respectively)
	//Pitch should be no greater than |30 degrees| (absolute value)
	//Roll should be no greater than 
	brandon.orientation = *orientCallback;
	
}	

// ---------- Functions ----------


/* void swing(int leg){

}*/

/* void stride(int leg){

}*/

// ------------- Main ------------

int main(int argc, char **argv){
	
	// define name of node and start
	ros::init(argc, argv, "gait_control");
    
	// The first nodehandle constructed will fully initialize this node
	ros::NodeHandle n;
    
	// specify loop frequency, works with Rate::sleep to sleep for the correct time
    ros::Rate loop_rate(20);

	// get ros params
	n.param("gait_control_enable_logging", enableLogging, false);
    
	// define topic name to publish to and queue size
	ros::Publisher gaitPub = n.advertise<fetch::RhoThetaQArray>("gait_control", 5);
    
	// define topic names to subscribe to and queue size
	ros::Subscriber switchSub = n.subscribe("foot_switches", 5, switchCallback);
	ros::Subscriber footSub = n.subscribe("foot_position", 5, footCallback);
	ros::Subscriber eulerSub = n.subscribe("orientation_control", 5, orientationControlCallback);
	//ros::Subscriber joystickSub = n.subscribe("joystick/xinput", 5, joystickCallback);

	while(ros::ok()){
		ros::spinOnce();


		gaitPub.publish(brandon.rtq);

        // The thing is, Bob, it's not that I'm lazy, it's that I just don't care.
		loop_rate.sleep();
	}

	return 0;
}

