#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Twist.h"
#include "fetch/RhoThetaQArray.h"
#include "std_msgs/UInt8MultiArray.h"
//#include "rc/mpu.h"

#include <string>



// -------- Variables --------

double FREQ;

double minRho, maxRho;
double idealRho, idealxOrient;
double forwardStabThresh, backwardStabThresh; //! add param
double defaultRho, defaultTheta, defaultQ;
double liftVel, dropVel;

bool enableLogging;
double leftRollThresh, rightRollThresh;
double forwardPitchThresh, backwardPitchThresh;
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
	float forward(float vel){
		if (vel >= 0) return plus;
		else return minus;
	}
	float reverse(float vel){
		if (vel >= 0) return minus;
		else return plus;
	}
};

class bounds{
	public:
	float plus;
	float minus;
	float forward(float vel){
		if (vel >= 0) return plus;
		else return minus;
	}
	float reverse(float vel){
		if (vel >= 0) return minus;
		else return plus;
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
	float k[4];
	int state[4];
	int legRef, nextLeg;

	int forwardLeg[2];
	int rearLeg[2];

	stabMargin stability;
	bounds e;
};

robot brandon;


// ---------- Functions ----------

int sign(float num){ return (num > 0) - (num < 0);}

void boundCalc(int leg, float rho, float theta){
	// make sure rho is an acceptable value first, limit it otherwise
	if (rho > maxRho) brandon.rtq.rho[leg] = maxRho; // upper limit
	if (rho < minRho) brandon.rtq.rho[leg] = minRho; // lower limit

	// for now just define them as needed
	// in the future this should have dynamic calculations for the bounds
	brandon.e.plus = 10;
	brandon.e.minus = -10;
}

stabMargin stabilityCalc(int testLegLift, int testLegDrop){
	// input what leg we are considering raising
	// if you want just the current state, set testLeg to -1 in the function call

	std_msgs::UInt8MultiArray localFootSw = brandon.footSwitch;
	geometry_msgs::Polygon localFootPos = brandon.footPosition;

	stabMargin stab;
	float sHolder = 0;
	stab.plus = 0;
	stab.minus = 0;

	// offset servos from center
	localFootPos.points[0].x += servoToCOM*cos(brandon.chassisXTheta);
	localFootPos.points[1].x += servoToCOM*cos(brandon.chassisXTheta);
	localFootPos.points[2].x -= servoToCOM*cos(brandon.chassisXTheta);
	localFootPos.points[3].x -= servoToCOM*cos(brandon.chassisXTheta);

	// assume test lift leg is in the air
	if(testLegLift != -1) localFootSw.data[testLegLift] = 0;

	// assume test lift leg is on the ground
	if(testLegDrop != -1) localFootSw.data[testLegDrop] = 1;

	for(int i=0;i<3;i++){

		int j;
		if (i < 3) j = i+1; // compare adjacent legs () legs on opposite sides of the robot L/R
		else j = 0; // loop back to compare first to last points

		if(localFootSw.data[i] != 0 && localFootSw.data[j] != 0){
			sHolder = (localFootPos.points[i].x + localFootPos.points[j].x) / 2;
			if(sHolder > stab.plus) stab.plus = sHolder;
			else if(sHolder < stab.minus) stab.minus = sHolder;
		}
	}
	return stab;
};

void orientAdjust(float xdir, float ydir){
	//* set x/y dir as 0 if not adjusting in that direction
	// note they are scaled to a lifting and dropping velocity (cm/sec)
	// sign points to direction adjusting TO, not the way you are falling
	//! + is right? - is left? ethan should confirm
	brandon.rtq.rho[0] += brandon.footSwitch.data[0] * (-xdir - ydir)/FREQ;
	brandon.rtq.rho[1] += brandon.footSwitch.data[1] * (-xdir + ydir)/FREQ;
	brandon.rtq.rho[2] += brandon.footSwitch.data[2] * (xdir - ydir)/FREQ;
	brandon.rtq.rho[3] += brandon.footSwitch.data[3] * (xdir + ydir)/FREQ;
	boundCalc(0,0,0);
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

	brandon.rtq.q[0] += brandon.footSwitch.data[0] * diff / FREQ;
	brandon.rtq.q[1] += brandon.footSwitch.data[1] * diff / FREQ;
	brandon.rtq.q[2] += brandon.footSwitch.data[2] * diff / FREQ;
	brandon.rtq.q[3] += brandon.footSwitch.data[3] * diff / FREQ;
	boundCalc(0,0,0);
}

void footZero(){
	for (int i =0; i < 4; i++){
		brandon.rtq.rho[i]=defaultRho;
		brandon.rtq.theta[i]=defaultTheta;
		brandon.rtq.q[i]=defaultQ;
	}
}

// ----- Callback Functions ------

void manualControlCallback(const geometry_msgs::Twist::ConstPtr& msg){
	// input velocity
	// add format of multiarray for ease-of-use
	brandon.velocity = *msg;	// velocity
	brandon.velocity.linear.x *= 50;
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
	
	if(brandon.orientation.x > forwardPitchThresh){
		orientAdjust(-1,0); 
	}

	if(brandon.orientation.x < backwardPitchThresh){
		orientAdjust(1,0);
	}

	if(brandon.orientation.y < leftRollThresh){
		orientAdjust(0,1);
	}

	if(brandon.orientation.y > rightRollThresh){
		orientAdjust(0,-1); 
	}
}	


// ------- State Functions -------

void lift(int leg){ //* state 0
	brandon.rtq.rho[leg] -= 5;
	brandon.state[leg] = 1;
	boundCalc(0,0,0);
}

void swing(int leg, float liftHeight){ //* state 1
	// ensure leg is lifted up to standard height
	if (brandon.rtq.rho[leg] > liftHeight){
		brandon.rtq.rho[leg] -= liftVel/FREQ;
		boundCalc(0,0,0);
	}else if (brandon.rtq.rho[leg] < liftHeight){
		brandon.rtq.rho[leg] = liftHeight; //todo adjust for frequency
		boundCalc(0,0,0);
	}
	// ensure leg is pulled to the right point on either side, depending on direction
	if (brandon.rtq.q[leg] < brandon.e.forward(brandon.velocity.linear.x)){
		brandon.rtq.q[leg] += 4*brandon.velocity.linear.x/FREQ;
	}else brandon.rtq.q[leg] = brandon.e.forward(brandon.velocity.linear.x);
}

void drop(int leg){ //* state 2
	if (brandon.footSwitch.data[leg] == false){
		brandon.rtq.rho[leg] += dropVel/FREQ;
		boundCalc(0,0,0);
	}else {
		brandon.state[leg] = 3;
		brandon.cycleDuration[leg] = ros::Time::now() - brandon.cycleStart[leg];
		brandon.cycleStart[leg] = ros::Time::now();
		brandon.legRef = leg;
	}
}

void stride(int leg){ //* state 3
	if (brandon.footSwitch.data[leg] == false) {
		brandon.rtq.rho[leg] += 1; 
		boundCalc(0,0,0);
	}
	brandon.rtq.q[leg] -= brandon.velocity.linear.x/FREQ;
	brandon.k[leg] = abs(brandon.rtq.q[leg] - brandon.e.reverse(brandon.velocity.linear.x));
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
	n.param("gait_control_frequency", FREQ, 20.0);
	n.param("gait_control_enable_logging", enableLogging, false);
	n.param("servo_to_com", servoToCOM, 18.5);
	n.param("min_chassis_rho", minRho, 10.0);
	n.param("max_chassis_rho", maxRho, 30.0);
	n.param("default_leg_rho", defaultRho, 20.0);
	n.param("default_leg_theta", defaultTheta, 0.0);
	n.param("default_leg_q", defaultQ, 0.0);
	n.param("default_chassis_rho", idealRho, 25.0);
	n.param("default_x_orient", idealxOrient, 0.0);
	n.getParam("leg_boundaries", legBounds);
	n.param("swing_velocity", liftVel, 30.0);
	n.param("drop_velocity", dropVel, 50.0);
   
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
	// initialize leg positions
	footZero();

	while(ros::ok()){
		ros::spinOnce();

		brandon.stability = stabilityCalc(-1,-1); // update stability margins at the beginning of each loop

		//TODO: decision making and state assignment goes here
		
		//! all decisions need to check stability first
		//*set priority list for reacting to certain conditions

		//*stability margin
		// S+
		if (brandon.stability.forward(brandon.velocity.linear.x) < forwardStabThresh){
			for(int i=0; i < 1; i ++){
				if (brandon.state[brandon.forwardLeg[i]] != 3){
					if (stabilityCalc(-1, brandon.forwardLeg[i]).forward(brandon.velocity.linear.x) > forwardStabThresh) brandon.state[brandon.forwardLeg[i]] = 2;
					else brandon.state[brandon.forwardLeg[i]] = 1;
				}
			}
		}

		// s- : 	only big issue with slope probably
		
		//*phase between legs
		// ... 0,2,1,3 ...
		// try to keep duty cycle/time delay between legs
		// elength = velocity * CT * DutyRatio
		//// DR = 1 - phi/ something?	
		//// (1-DR)*avCT
		// compare average cycle time and start times
		switch (brandon.legRef){
			case 0:
			brandon.nextLeg = 2;
			case 1:
			brandon.nextLeg = 3;
			case 2:
			brandon.nextLeg = 1;
			case 3:
			brandon.nextLeg = 0;
		}
		if(stabilityCalc(brandon.nextLeg, -1).min() > forwardStabThresh) brandon.state[brandon.nextLeg] = 0;
		
		//*k check
		//	final check, otherwise no state changes necessary
		for (int i=0;i<4;i++){
			if (brandon.state[i] == 3 && brandon.k[i] < 2){
				int legCheck = 0;
				for(int j=0; j<4; j++){
					if (brandon.state[j] !=3) legCheck = 1;
					}
				if(legCheck = 1 && stabilityCalc(i, -1).min()) brandon.state[i] = 0;
			}
		}

		for (int i=0;i<4;i++){
			switch(brandon.state[i]){

			case 0: // do you even lift bro?
				lift(i);

			case 1: // Schwing!
				swing(i, 10);

			case 2:
				drop(i); // Don't drop the soap, drop your foot
			
			case 3: // Running on a dream.
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

