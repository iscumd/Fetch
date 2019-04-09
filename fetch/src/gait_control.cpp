#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Twist.h"
#include "fetch/RhoThetaQArray.h"
#include "std_msgs/UInt8MultiArray.h"
//#include "rc/mpu.h"

#include <string>

#define FREQ 20


// -------- Variables --------

double idealRho;
double idealxOrient;

bool enableLogging;
double leftRollThresh, rightRollThresh, forwardPitchThresh;
double backwardPitchThresh;
double servoToCOM;
std::vector<double> legBounds;


// ----------- Classes -----------

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
	
	ros::Time cycleStart[4];
	ros::Duration strideDuration[4];
	ros::Duration cycleDuration[4];

	float chassisRho;
	float chassisXTheta;

	fetch::RhoThetaQArray rtq;
	float deltaRho[4];
	int state[4];

	stabMargin stability(int testLegLift, int testLegDrop){
		// input what leg we are considering raising
		// if you want just the current state, set testLeg to -1 in the function call

		std_msgs::UInt8MultiArray localFootSw = footSwitch;
		geometry_msgs::Polygon localFootPos = footPosition;

		float theta = chassisXTheta;

		stabMargin stab;
		float sHolder = 0;
		stab.plus = 0;
		stab.minus = 0;

		// offset servos from center
		localFootPos.points[0].x += servoToCOM*cos(theta);
		localFootPos.points[1].x += servoToCOM*cos(theta);
		localFootPos.points[2].x -= servoToCOM*cos(theta);
		localFootPos.points[3].x -= servoToCOM*cos(theta);

		// set test lift leg to zero
		if(testLegLift != -1) localFootSw.data[testLegLift] = 0;
		// set test drop leg to one
		if(testLegDrop != -1) localFootSw.data[testLegDrop] = 1;

		for(int i=0;i<3;i++){
			//TODO adjust for servo position with respect to COM and preferably orientation, currently just applies based on center which is wrong
			if(localFootSw.data[i] != 0 && localFootSw.data[i+1] != 0){
				sHolder = (localFootPos.points[i].x + localFootPos.points[i+1].x) / 2;
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
	//TODO make boundary calculations a function in gait_control
	
	//Message reads in Pitch, Roll, Yaw(msg->data.x, msg->data.y, msg->data.z respectively)
	//Pitch should be no greater than |30 degrees| (absolute value)
	//Roll should be no greater than 
	brandon.orientation = *msg;
	
	//Units in cm for deltaRho
	//Will adjust the deltaRho offsets for each leg based on the offset data to adjust balance
	if(brandon.orientation.x > forwardPitchThresh){
		////orientAdjust(-1,0,1);
		brandon.deltaRho[0]--;
		brandon.deltaRho[1]--;
		brandon.deltaRho[2]++;
		brandon.deltaRho[3]++;
	}

	if(brandon.orientation.x < backwardPitchThresh){
		////orientAdjust(1,0,1);
		brandon.deltaRho[0]++;
		brandon.deltaRho[1]++;
		brandon.deltaRho[2]--;
		brandon.deltaRho[3]--;
	}

	if(brandon.orientation.y < leftRollThresh){
		////orientAdjust(0,1,1);
		brandon.deltaRho[0]--;
		brandon.deltaRho[2]--;
		brandon.deltaRho[1]++;
		brandon.deltaRho[3]++;
	}

	if(brandon.orientation.y > rightRollThresh){
		////orientAdjust(0,-1,1);
		brandon.deltaRho[0]++;
		brandon.deltaRho[2]++;
		brandon.deltaRho[1]--;
		brandon.deltaRho[3]--;
	}
}	

// ---------- Functions ----------

int sign(float num){ return (num > 0) - (num < 0);}

void orientAdjust(float xdir, float ydir, float magnitude){
	//* set x/y dir as 0 if not adjusting in that direction
	// sign points to direction adjusting TO, not the way you are falling
	//! + is right? - is left? ethan should confirm
	brandon.rtq.rho[0] += magnitude * (-xdir - ydir);
	brandon.rtq.rho[1] += magnitude * (-xdir + ydir);
	brandon.rtq.rho[2] += magnitude * (xdir - ydir);
	brandon.rtq.rho[3] += magnitude * (xdir + ydir);
	//! bounds function call needed
}

void heightAdjust(float height){
	int legCount = 0;
	float avHeight = 0;
	float diff;
	for (int i; i<4; i++){
		if (brandon.footSwitch.data[i] == 1){
			legCount++ ;
			avHeight += brandon.rtq.rho[i];
		}
	}
	avHeight = avHeight/legCount;
	diff = height - avHeight;

	brandon.rtq.q[0] += brandon.footSwitch.data[0] * diff / 2;
	brandon.rtq.q[1] += brandon.footSwitch.data[1] * diff / 2;
	brandon.rtq.q[2] += brandon.footSwitch.data[2] * diff / 2;
	brandon.rtq.q[3] += brandon.footSwitch.data[3] * diff / 2;
	//! bounds function call needed
}

void lift(int leg){ //* state 0
	brandon.rtq.rho[leg] -= 5;
	////brandon.deltaRho[leg] -= 5;
	brandon.state[leg] = 1;
	//! bounds function call needed
}

void swing(int leg, float liftHeight){ //* state 1
	// ensure leg is lifted up to standard height
	if (brandon.rtq.rho[leg] > liftHeight){
		brandon.rtq.rho[leg] -= 2;
	}else brandon.rtq.rho[leg] = liftHeight;
	//! bounds function call needed
	// ensure leg is pulled to the right point on either side, depending on direction
	if (brandon.rtq.q[leg] < 10*sign(brandon.velocity.linear.x)){
		brandon.rtq.q[leg] += 3*brandon.velocity.linear.x;
	}else brandon.rtq.q[leg] = 10*sign(brandon.velocity.linear.x);
}

void drop(int leg){ //* state 2
	if (brandon.footSwitch.data[leg] == false){
		brandon.rtq.rho[leg] += 1;
		////brandon.deltaRho[leg] +=1;
		//! bounds function call needed
	}else {
		brandon.state[leg] = 3;
		brandon.cycleDuration[leg] = ros::Time::now() - brandon.cycleStart[leg];
		brandon.cycleStart[leg] = ros::Time::now();
		//TODO add timestamps per leg?
	}
}

void stride(int leg){ //* state 3
	if (brandon.footSwitch.data[leg] == false) brandon.rtq.rho[leg] += 1; 
	//! bounds function call needed if rho is changed
	////brandon.deltaRho[leg] +=1;
	brandon.rtq.q[leg] -= brandon.velocity.linear.x/FREQ;
	// TODO set up boundaries in brandon class
	////if (brandon.rtq.q[leg] - brandon.e[leg].minus < 2) brandon.
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
	n.param("servo_to_com", servoToCOM, 18.5); //! default value should be measured
	n.param("default_chassis_rho", idealRho, 25.0);
	n.param("default_x_orient", idealxOrient, 0.0);
	n.getParam("leg_boundaries", legBounds); //! not included in launch yet but really really needs to be there
   
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

		//TODO: decision making and state assignment goes here
		//set priority list for reacting to certain conditions
		

		for (int i=0;i<4;i++){
			switch(brandon.state[i]){
				//* 0 for swing function 1 for stride function

			case 0:
				lift(i);

			case 1:
				swing(i, 10);

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

