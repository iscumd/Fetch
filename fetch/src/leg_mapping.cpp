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
	geometry_msgs::Polygon footPoint;			// point output
	std_msgs::Float32MultiArray servoAngle;		// angle output

	double rho;
	double legTheta;
	double thetaOffset;
	int i;

    fetch::RhoThetaQArray rtq = *msg;	// rtq input
	// Convert rtq to x,z for each leg
	for(int i = 0; i < 4; i++){
		geometry_msgs::Point32 point;

		// rtq forms a right triangle
		rho = sqrt(rtq.q[i]*rtq.q[i] + rtq.rho[i]*rtq.rho[i]); // magnitude of high-on-potenuse
		legTheta = atan(rtq.q[i] / rtq.rho[i]) + rtq.theta[i];	// angle of triangle

		point.x = rho*sin(legTheta);
		point.z = -rho*cos(legTheta);  // negative because otherwise rho will have to be negative and that just seems weird

		if(enableLogging) ROS_INFO("leg mapping calculated points for leg [%i]\tx = [%f]\tz = [%f]", i, point.x, point.z);
		footPoint.points.push_back(point);
	}; 
	footPointPub.publish(footPoint);

	//  Convert x,z to theta1, theta2 for each leg
	for(int i = 0; i < 4; i++){
		// simple cartesian to polar coordinate conversion
		rho = sqrt(footPoint.points[i].x*footPoint.points[i].x + footPoint.points[i].z*footPoint.points[i].z);
		legTheta = atan(footPoint.points[i].z / footPoint.points[i].x);

		// law of cosines to determine angles of the triangle
		thetaOffset = acos(((upperLeg*upperLeg + rho*rho - lowerLeg*lowerLeg) / (2*upperLeg*rho))/PI);

		//* angles are simply the polar angle +/- the angle of the triangle formed

		// 'front' leg angle
		float frontAngle;
		frontAngle = (legTheta + thetaOffset) * 180 / PI; 
		frontAngle = normalizeAngle(frontAngle, 90, 180); // TODO: set up servo angle origin params
		servoAngle.data.push_back(frontAngle);

		// 'back' leg angle
		float backAngle;
		backAngle = (legTheta - thetaOffset) * 180 / PI;
		backAngle = normalizeAngle(backAngle, 270, 180); // TODO: set up servo angle origin params
		servoAngle.data.push_back(backAngle);

		if(enableLogging){ROS_INFO("leg mapping calculated angles for leg [%i]\tfront = [%f]\trear = [%f]", i, frontAngle, backAngle); };
	};
	servoAnglePub.publish(servoAngle);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "leg_mapping");

	ros::NodeHandle n;

	n.param("leg_mapping_enable_logging", enableLogging, false);
	n.param("leg_mapping_upper_leg", upperLeg, 12.7);
	n.param("leg_mapping_lower_leg", lowerLeg, 25.4);

	servoAnglePub = n.advertise<std_msgs::Float32MultiArray>("leg_mapping", 5);
	footPointPub = n.advertise<geometry_msgs::Polygon>("foot_position", 5);

	ros::Subscriber rtqSub = n.subscribe("gait_control", 5, rtqCallback);

	ros::spin();

	return 0;
}
