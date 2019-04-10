#include "ros/ros.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point32.h"
#include "std_msgs/Float32.h"
#include "fetch/RhoThetaQArray.h"
#include "std_msgs/Float32MultiArray.h"

#include <cmath>
#include <string>

#define PI 3.14159265

ros::Publisher servoAnglePub;
ros::Publisher footPointPub;

// variables

// params
double upperLeg;
double lowerLeg;
bool enableLogging;

float normalizeAngle(float angle, float min, float range){
	//! needs verification
	
	while (angle < 0) angle += 360; // remove negative options

	angle -= min; // shift origin to the starting point

	if (angle < 0 ) angle = 0; // filter negative out of bounds
	else if (angle > range) angle = range; // filter positive out of bounds

	return angle;
}

void rtqCallback(const fetch::RhoThetaQArray::ConstPtr& msg)
{
	ros::Time startTime =ros::Time::now();

	geometry_msgs::Polygon footPoint;			// point output
	std_msgs::Float32MultiArray servoAngle;		// angle output

	double legRho;
	double legTheta;
	double thetaOffset;
	int i;

    fetch::RhoThetaQArray rtq = *msg;	// rtq input
	// Convert rtq to x,z for each leg
	for(int i = 0; i < 4; i++){
		geometry_msgs::Point32 point;

		// rtq forms a right triangle
		legRho = sqrt(rtq.q[i]*rtq.q[i] + rtq.rho[i]*rtq.rho[i]); // magnitude of high-on-potenuse
		legTheta = atan2(rtq.q[i], rtq.rho[i]) + rtq.theta[i] + 3*PI/2;	// angle of triangle

		//* angles are simply the polar angle +/- the angle of the triangle formed
		thetaOffset = acos(((upperLeg*upperLeg + legRho*legRho - lowerLeg*lowerLeg) / (2*upperLeg*legRho))/PI);

		point.x = legRho*cos(legTheta);
		point.z = legRho*sin(legTheta);
		footPoint.points.push_back(point);

		// 'front' leg angle
		float frontAngle;
		frontAngle = (legTheta + thetaOffset) * 180 / PI; 
		if(enableLogging) ROS_INFO("LM:\tleg [%i]\tthetaf:[%f]", i, frontAngle);
		frontAngle = normalizeAngle(frontAngle, 240, 180); // TODO: set up servo angle origin params
		servoAngle.data.push_back(frontAngle);

		// 'back' leg angle
		float backAngle;
		backAngle = (legTheta - thetaOffset) * 180 / PI;
		if(enableLogging) ROS_INFO("LM:\tleg [%i]\tthetab:[%f]", i, backAngle);
		backAngle = normalizeAngle(backAngle, 120, 180); // TODO: set up servo angle origin params
		servoAngle.data.push_back(backAngle);

		if(enableLogging) ROS_INFO("LM:\tleg [%i]\tpoints:\tx:[%f]\tz:[%f]", i, point.x, point.z);
		if(enableLogging){ROS_INFO("LM:\tleg [%i]\tangles:\tfront:[%f]\trear:[%f]", i, frontAngle, backAngle); };
	}; 
	footPointPub.publish(footPoint);
	servoAnglePub.publish(servoAngle);
 
	if(enableLogging){
		ros::Duration d = ros::Time::now() - startTime;
   		double secs = d.toSec();
		ROS_INFO("LM:\texec_time:\t[%f]", secs);
		}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "leg_mapping");

	ros::NodeHandle n;

	n.param("leg_mapping_enable_logging", enableLogging, true);
	n.param("leg_mapping_upper_leg", upperLeg, 12.7);
	n.param("leg_mapping_lower_leg", lowerLeg, 25.4);

	servoAnglePub = n.advertise<std_msgs::Float32MultiArray>("leg_mapping", 5);
	footPointPub = n.advertise<geometry_msgs::Polygon>("foot_position", 5);

	ros::Subscriber rtqSub = n.subscribe("gait_control", 5, rtqCallback);

	ros::spin();

	return 0;
}
