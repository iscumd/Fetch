#include "ros/ros.h"
#include "geometry_msgs/Polygon.h"
#include "std_msgs/Float32.h"
#include "fetch/RhoThetaQArray.h"
#include "std_msgs/Float32MultiArray.h"

#include <cmath>
#include <string>

#define PI 3.14159265

ros::Publisher servoAnglePub;
ros::Publisher footPointPub;

// variables
double rho;
double theta;
double thetaOffset;
int i;

// params
double upperLeg;
double lowerLeg;
bool enableLogging;

void rtqCallback(const fetch::RhoThetaQArray::ConstPtr& rtqCallback)
{

	geometry_msgs::Polygon footPoint;			// point output
	std_msgs::Float32MultiArray servoAngle;		// angle output

    fetch::RhoThetaQArray rtq = *rtqCallback;	// rtq input

	// Convert rtq to x,z for each leg
	// refer to picture of shit on the drive
	for(int i = 0; i < 4; i++){
		rho = sqrt(rtq.q[i]*rtq.q[i] + rtq.rho[i]*rtq.rho[i]);
		theta = atan(rtq.q[i] / rtq.rho[i]) + rtq.theta[i];
		footPoint.points[i].x = rho*cos(theta);
		footPoint.points[i].z = rho*sin(theta);
		if(enableLogging){ROS_INFO("leg mapping calculated points for leg [%i] \t x = [%f] \t z = [%f]", i, footPoint.points[i].x, footPoint.points[i].z); };
	}; 

	footPointPub.publish(footPoint);

	// Convert x,z to theta1, theta2 for each leg
	// refer to legstuff.m on Drive
	for(int i = 0; i < 4; i++){
		rho = sqrt(footPoint.points[i].x*footPoint.points[i].x + footPoint.points[i].z*footPoint.points[i].z);
		theta = atan(footPoint.points[i].y / footPoint.points[i].x);
		thetaOffset = acos((upperLeg*upperLeg + rho*rho - lowerLeg*lowerLeg) / (2*upperLeg*rho));
		servoAngle.data[2*i] = (theta + thetaOffset) * PI / 180; 	// 'front' leg angle
		servoAngle.data[2*i+1] = (theta - thetaOffset) * PI / 180;	// 'rear' leg angle

		if(enableLogging){ROS_INFO("leg mapping calculated angles for leg [%i] \t front = [%f] \t rear = [%f]", i, servoAngle.data[2*i], servoAngle.data[2*i+1]); };
	};
	 
	servoAnglePub.publish(servoAngle);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "leg_mapping");

	ros::NodeHandle n;

	n.param("leg_mapping_enable_logging", enableLogging, false);
	n.param("leg_mapping_upper_leg", upperLeg, 0.127);
	n.param("leg_mapping_lower_leg", lowerLeg, 0.254);

	servoAnglePub = n.advertise<std_msgs::Float32MultiArray>("leg_mapping", 5);
	footPointPub = n.advertise<geometry_msgs::Polygon>("foot_position", 5);

	ros::Subscriber rtqSub = n.subscribe("joystick/xinput", 5, rtqCallback);

	ros::spin();

	return 0;
}
