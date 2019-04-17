#include "ros/ros.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Twist.h"
#include "fetch/RhoThetaQArray.h"
#include "fetch/OrientationRPY.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Bool.h"
//#include "rc/mpu.h"

#include <string>

#define LIFT 0
#define SWING 1
#define DROP 2
#define STRIDE 3

// -------- Variables --------

double FREQ;

double minRho, maxRho;
double innerE, outerE;
double stabilityThreshold, backwardStabilityThreshold;
double defaultRho, defaultTheta, defaultQ;
double liftVel, dropVel, maxVel;

bool enableLogging;
double leftRollThresh, rightRollThresh;
double forwardPitchThresh, backwardPitchThresh;
double servoToCOM;
std::vector<double> legBounds;
ros::Publisher gaitPub;


// ----------- Classes -----------

class stabMargin{
public:
	float plus, minus;
	float min(void){
		if(abs(plus) < abs(minus)) return abs(plus);
		else return abs(minus); 
	}
	float forward(float vel){
		if (vel >= 0) {
			return plus;
			if(enableLogging) ROS_INFO("stability margin forward returned plus");
		}
		else
		{
			return minus;
			if(enableLogging) ROS_INFO("stability margin forward returned minus");
		}
	}
	float reverse(float vel){
		if (vel < 0) {
			return plus;
			if(enableLogging) ROS_INFO("stability margin reverse returned plus");
		}
		else
		{
			return minus;
			if(enableLogging) ROS_INFO("stability margin reverse returned minus");
		}
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
	fetch::OrientationRPY orientation;
	geometry_msgs::Twist velocity;
	
	ros::Time cycleStart[4];
	ros::Duration strideDuration[4];
	ros::Duration cycleDuration[4];

	float chassisRho;
	float chassisXTheta;

	fetch::RhoThetaQArray rtq;
	float k[4];
	int state[4];
	std::vector<int> legPattern;
	int lastLeg, nextLeg;

	int forwardLeg[2];
	int rearLeg[2];

	stabMargin stability;
	bounds e[4];

	robot() {
		footSwitch = std_msgs::UInt8MultiArray();
		orientation = fetch::OrientationRPY();
		velocity = geometry_msgs::Twist();
		chassisXTheta = defaultTheta;
		chassisRho = defaultRho;
		lastLeg = 0;
		stability = stabMargin();
		legPattern = {1,2,3,0};
	}
};

robot brandon = robot();


// ---------- Functions ----------

int sign(float num){ return (num > 0) - (num < 0);}

int legCheck(int testLeg){
	// set testLeg to leg intended to be lifted
	// intended to check that all other legs are on the ground first / in stride state
	int legCheck = 0;
	for(int leg =0; leg<4; leg++){
		if (brandon.state[leg] != STRIDE && leg != testLeg)	legCheck = 1;
	}
	return legCheck;
}

void boundCalc(){
	for (int leg = 0; leg < 4; leg++){
		// make sure rho is an acceptable value first, limit it otherwise
		if (brandon.rtq.rho[leg] > maxRho) brandon.rtq.rho[leg] = maxRho; // upper limit
		if (brandon.rtq.rho[leg] < minRho) brandon.rtq.rho[leg] = minRho; // lower limit

		// for now just define them as needed
		// in the future this should have dynamic calculations for the bounds
		if (leg <= 1){
		brandon.e[leg].plus = outerE;
		brandon.e[leg].minus = -innerE;
		}else{
		brandon.e[leg].plus = innerE;
		brandon.e[leg].minus = -outerE;
		}
	}
}

stabMargin stabilityCalc(int testLegLift, int testLegDrop){
	// input what leg we are considering raising
	// if you want just the current state, set testLeg to -1 in the function call

	std_msgs::UInt8MultiArray localFootSw = brandon.footSwitch;
	fetch::RhoThetaQArray localRTQ = brandon.rtq;

	stabMargin stab;
	float sHolder = 0;
	stab.plus = 0;
	stab.minus = 0;

	// offset servos from center of mass (COM)
	localRTQ.q[0] += servoToCOM*cos(brandon.chassisXTheta);
	localRTQ.q[1] += servoToCOM*cos(brandon.chassisXTheta);
	localRTQ.q[2] -= servoToCOM*cos(brandon.chassisXTheta);
	localRTQ.q[3] -= servoToCOM*cos(brandon.chassisXTheta);

	// assume test lift leg is in the air
	if(testLegLift != -1) {
		localFootSw.data[testLegLift] = 0;
		if(enableLogging) ROS_INFO("GC:\tstabilityCalc\tlift leg:\t[%i]", testLegLift);
	}

	// assume test lift leg is on the ground
	if(testLegDrop != -1) {
		localFootSw.data[testLegDrop] = 1;
		if(enableLogging) ROS_INFO("GC:\tstabilityCalc\tdrop leg:\t[%i]", testLegDrop);
	}

	for(int i=0;i<3;i++){

		int j;
		if (i < 3) j = i+1; // compare adjacent legs () legs on opposite sides of the robot L/R
		else j = 0; // loop back to compare first to last points

		if(localFootSw.data[i] != 0 && localFootSw.data[j] != 0){
			sHolder = (localRTQ.q[i] + localRTQ.q[j]) / 2;
			if(sHolder > stab.plus) stab.plus = sHolder;
			else if(sHolder < stab.minus) stab.minus = sHolder;
		}
	}
	if(enableLogging) ROS_INFO("GC:\tstabilityCalc\tplus:\t[%f]\tminus:\t[%f]\tforward:\t[%f]\treverse:\t[%f]", stab.plus, stab.minus, stab.forward(brandon.velocity.linear.x), stab.reverse(brandon.velocity.linear.x));
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
	if(enableLogging) ROS_INFO("GC:\torientAdjust\txdir:\t[%f]\tydir:\t[%f]", xdir, ydir);
	boundCalc();
}

void heightAdjust(float height){
	int legCount = 0;
	float avHeight = 0;
	float diff;
	for (int i = 0; i<4; i++){
		if (brandon.footSwitch.data[i] == true){
			legCount++ ;
			avHeight += brandon.rtq.rho[i];
		}
	}
	if (legCount == 0){
		if(enableLogging) ROS_INFO("GC:\theightAdjust\tno adjust necessary");
	}else{
		avHeight = avHeight/legCount;
		diff = height - avHeight;
		if(enableLogging) ROS_INFO("GC:\theightAdjust\tdiff:\t[%f]\tdelta:\t[%f]", diff, diff / FREQ);

		brandon.rtq.rho[0] += brandon.footSwitch.data[0] * diff / FREQ;
		brandon.rtq.rho[1] += brandon.footSwitch.data[1] * diff / FREQ;
		brandon.rtq.rho[2] += brandon.footSwitch.data[2] * diff / FREQ;
		brandon.rtq.rho[3] += brandon.footSwitch.data[3] * diff / FREQ;
		boundCalc();
	}
}

void footInitialize(){
	ROS_INFO("footInitialize");
	brandon.rtq.rho = std::vector<float>();
	brandon.rtq.theta = std::vector<float>();
	brandon.rtq.q = std::vector<float>();
	// intialize rtq
	for (int i = 0; i < 4; i++){
		brandon.rtq.rho.push_back(defaultRho);
		brandon.rtq.theta.push_back(defaultTheta);
		brandon.rtq.q.push_back(defaultQ);
		brandon.state[i] = STRIDE;
	}
	// set initial phases
	float strideLength = outerE + innerE;
	for (int i = 0; i < 4; i++){
		float phase = strideLength/(4-i);
		brandon.rtq.q.at(brandon.legPattern.at(i)) = phase + brandon.e[brandon.legPattern.at(i)].minus;
	}
	brandon.lastLeg = 0;
}

// ----- Callback Functions ------

void manualControlCallback(const geometry_msgs::Twist::ConstPtr& msg){
	// input velocity
	// add format of multiarray for ease-of-use
	brandon.velocity = *msg;	// velocity
	brandon.velocity.linear.x *= maxVel;
	if(enableLogging) ROS_INFO("GC:\tcontrollerCB\tlinear x:\t[%f] cm/s", brandon.velocity.linear.x);
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

void orientationControlCallback(const fetch::OrientationRPY::ConstPtr& msg){
	//Pitch should be no greater than |30 degrees| (absolute value)
	//Roll should be no greater than 
	brandon.orientation = *msg;
	
	if(brandon.orientation.pitch > forwardPitchThresh){
		if(enableLogging) ROS_INFO("GC:\torientCB\tforwardPitchThresh exceeded:\tpitch:\t[%f]", brandon.orientation.pitch);
		orientAdjust(-1,0); 
	}

	if(brandon.orientation.pitch < backwardPitchThresh){
		if(enableLogging) ROS_INFO("GC:\torientCB\tbackwardPitchThresh exceeded:\tpitch:\t[%f]", brandon.orientation.pitch);
		orientAdjust(1,0);
	}

	if(brandon.orientation.roll < leftRollThresh){
		if(enableLogging) ROS_INFO("GC:\torientCB\tleftRollThresh exceeded:\troll:\t[%f]", brandon.orientation.roll);
		orientAdjust(0,1);
	}

	if(brandon.orientation.roll > rightRollThresh){
		if(enableLogging) ROS_INFO("GC:\torientCB\trightRollThresh exceeded:\troll:\t[%f]", brandon.orientation.roll);
		orientAdjust(0,-1); 
	}
}

void recenterCallback(const std_msgs::Bool::ConstPtr& msg){
	if (msg->data == true) {
		footInitialize();
		gaitPub.publish(brandon.rtq);
		ros::Duration(3).sleep();
	}
}


// ------- State Functions -------

void lift(int leg){ //* state 0
	brandon.rtq.rho[leg] -= 5;
	brandon.state[leg] = SWING;
	boundCalc();
	//if(enableLogging) ROS_INFO("GC:\tlift state\tleg:\t[%i]", leg);
}

void swing(int leg, float liftHeight){ //* state 1
	// ensure leg is lifted up to standard height
	if (brandon.rtq.rho[leg] > liftHeight){ // tries to lift to desired height
		brandon.rtq.rho[leg] -= liftVel/FREQ;
		boundCalc();
		//if(enableLogging) ROS_INFO("GC:\tswing state\tleg:\t[%i]\tlifting to:\t[%f]\tdelta:", leg, liftHeight, liftVel/FREQ);
	}
	else if (brandon.rtq.rho[leg] < liftHeight){ // ensures leg doesnt go over desired height
		brandon.rtq.rho[leg] = liftHeight;
		boundCalc();
	}
	//else{ // just tells us we've reached the desired height
		//if(enableLogging) ROS_INFO("GC:\tswing state\tleg:\t[%i]\treached liftHeight: [%f]", leg, liftHeight);
	//}

	// ensure leg is pulled to the right point on either side, depending on direction
	if (brandon.rtq.q[leg] < brandon.e[leg].forward(brandon.velocity.linear.x)){
		brandon.rtq.q[leg] += 4*brandon.velocity.linear.x/FREQ;
		//if(enableLogging) ROS_INFO("GC:\tswing state\tleg:\t[%i]\treached liftHeight: [%f]", leg, liftHeight);

	}else
	{
		brandon.rtq.q[leg] = brandon.e[leg].forward(brandon.velocity.linear.x);
		brandon.state[leg] = DROP;
	}
}

void drop(int leg){ //* state 2
	if (brandon.footSwitch.data[leg] == false && brandon.rtq.rho[leg] != maxRho){
		brandon.rtq.rho[leg] += dropVel/FREQ;
		boundCalc();
	}else {
		brandon.state[leg] = STRIDE;
		brandon.cycleDuration[leg] = ros::Time::now() - brandon.cycleStart[leg];
		brandon.cycleStart[leg] = ros::Time::now();
		brandon.lastLeg = leg;
	}
}

void stride(int leg){ //* state 3
	if (brandon.footSwitch.data[leg] == false){
		brandon.rtq.rho[leg] += 1;
		//ros::Duration(0.02).sleep();
	}
	boundCalc();
	
	brandon.rtq.q[leg] -= brandon.velocity.linear.x/FREQ;
	brandon.k[leg] = abs(brandon.rtq.q[leg] - brandon.e[leg].reverse(brandon.velocity.linear.x));
	if(brandon.k[leg] < 2 && legCheck(leg) == 0) brandon.state[leg] = LIFT; // lift if you hit the very rear bound and no other legs are lifted
}

// ------------- Main ------------

int main(int argc, char **argv){
	
	// define name of node and start
	ros::init(argc, argv, "gait_control");
    
	// The first nodehandle constructed will fully initialize this node
	ros::NodeHandle n;

	// get ros params
	n.param("gait_control_frequency", FREQ, 20.0);
	n.param("gait_control_enable_logging", enableLogging, false);
	n.param("servo_to_com", servoToCOM, 18.5);
	n.param("min_chassis_rho", minRho, 12.0);
	n.param("max_chassis_rho", maxRho, 30.0);
	n.param("outer_e_bound", outerE, 10.0);
	n.param("inner_e_bound", innerE, 8.0);
	n.param("default_leg_rho", defaultRho, 20.0);
	n.param("default_leg_theta", defaultTheta, 0.0);
	n.param("default_leg_q", defaultQ, 0.0);
	n.getParam("leg_boundaries", legBounds);
	n.param("swing_velocity", liftVel, 20.0);
	n.param("drop_velocity", dropVel, 20.0);
	n.param("max_velocity", maxVel, 15.0);
	n.param("forward_stability_threshold", stabilityThreshold, 5.0);
   
    //Pitch and Roll Thresholds.
	n.param("left_roll_threshold", leftRollThresh, -30.0);
	n.param("right_roll_threshold", rightRollThresh, 30.0);
	n.param("forward_pitch_threshold", forwardPitchThresh, 30.0);
	n.param("backward_pitch_threshold", backwardPitchThresh, -30.0);

	// define topic name to publish to and queue size
	gaitPub = n.advertise<fetch::RhoThetaQArray>("gait_control", 5);
    
	// define topic names to subscribe to and queue size
	ros::Subscriber switchSub = n.subscribe("foot_switches", 5, switchCallback);
	ros::Subscriber footSub = n.subscribe("foot_position", 5, footCallback);
	ros::Subscriber eulerSub = n.subscribe("orientation_control", 5, orientationControlCallback);
	ros::Subscriber manualControlSub = n.subscribe("manual_control", 5, manualControlCallback);
	ros::Subscriber servoRecenterSub = n.subscribe("gait_control_reinitialize", 5, recenterCallback);
    
	// specify loop frequency, works with Rate::sleep to sleep for the correct time
    ros::Rate loop_rate(FREQ);

	// initialize leg positions
	footInitialize();
	ros::Duration(2).sleep();

	while(ros::ok()){
		ros::spinOnce();

		if (brandon.footSwitch.data.empty()) {
			continue;
		}

		if (abs(brandon.velocity.linear.x) < 5){
			if(enableLogging) ROS_INFO("GC:\tno velocity given, no change");
			for(int i = 0; i<4; i++) {
				if (brandon.footSwitch.data.at(i) == false) brandon.state[i] = STRIDE;
			}
		}
		else
		{
			if(enableLogging) ROS_INFO("GC:\treached 'phase' check");	
			
			// ... 0,2,1,3 ...
			// simply sets the pattern by looking at last leg to 'drop' and selecting a leg to lift based on that
			if(enableLogging) ROS_INFO("GC:\t'phase' check\tlastLeg\t[%i]", brandon.lastLeg);
			
			// just moves onto the next leg
			brandon.nextLeg = brandon.legPattern.at(brandon.lastLeg);

			if(enableLogging) ROS_INFO("GC:\t'gait' check\tnextLeg\t[%i]", brandon.nextLeg);
			
			if(legCheck(brandon.nextLeg) == 0 && brandon.state[brandon.nextLeg] == STRIDE){ // only lift next leg if no other leg is lifted
				brandon.state[brandon.nextLeg] = LIFT;
				if(enableLogging) ROS_INFO("GC:\t'gait' check\tlifting leg\t[%i]", brandon.nextLeg);
			}
		};
		// act on set states for each leg
		for (int i=0;i<4;i++){
			switch(brandon.state[i]){

			case LIFT: // do you even lift bro?
				lift(i);
				if(enableLogging) ROS_INFO("GC:\tlifting leg \t[%i]", i);
				break;

			case SWING: // Schwing!
				swing(i, minRho);
				if(enableLogging) ROS_INFO("GC:\tswinging leg \t[%i]", i);
				break;

			case DROP:
				drop(i); // Don't drop the soap, drop your foot
				if(enableLogging) ROS_INFO("GC:\tdropping leg \t[%i]", i);
				break;
			
			case STRIDE: // Running on a dream.
				stride(i);
				if(enableLogging) ROS_INFO("GC:\tstriding leg \t[%i]", i);
				break;

			};
		};
		gaitPub.publish(brandon.rtq);

		if(enableLogging) ROS_INFO("Execution time: [%f]", loop_rate.cycleTime().toSec());
        
		// The thing is, Bob, it's not that I'm lazy, it's that I just don't care.
		loop_rate.sleep();
	}

	return 0;
}

