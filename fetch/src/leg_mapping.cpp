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

void rtqCallback(const fetch::RhoThetaQArray::ConstPtr& msg)
{
	geometry_msgs::Polygon footPoint;			// point output
	std_msgs::Float32MultiArray servoAngle;		// angle output

	double rho;
	double theta;
	double thetaOffset;
	int i;

    fetch::RhoThetaQArray rtq = *msg;	// rtq input
	ROS_INFO("1");
	// Convert rtq to x,z for each leg
	// refer to picture of shit on the drive
	for(int i = 0; i < 4; i++){
		geometry_msgs::Point32 point;

		rho = sqrt(rtq.q[i]*rtq.q[i] + rtq.rho[i]*rtq.rho[i]);
		theta = atan(rtq.q[i] / rtq.rho[i]) + rtq.theta[i];

		point.x = rho*cos(theta);
		point.z = rho*sin(theta);

		if(enableLogging) ROS_INFO("leg mapping calculated points for leg [%i] \t x = [%f] \t z = [%f]", i, point.x, point.z);
		footPoint.points.push_back(point);
	}; 
	footPointPub.publish(footPoint);

	// Convert x,z to theta1, theta2 for each leg
	// refer to legstuff.m on Drive
	for(int i = 0; i < 4; i++){
		rho = sqrt(footPoint.points[i].x*footPoint.points[i].x + footPoint.points[i].z*footPoint.points[i].z);
		theta = atan(footPoint.points[i].y / footPoint.points[i].x);
		thetaOffset = acos((upperLeg*upperLeg + rho*rho - lowerLeg*lowerLeg) / (2*upperLeg*rho));

		float frontAngle;
		frontAngle = (theta + thetaOffset) * PI / 180; 	// 'front' leg angle
		servoAngle.data.push_back(frontAngle);

		float backAngle;
		backAngle = (theta - thetaOffset) * PI / 180;	// 'rear' leg angle
		servoAngle.data.push_back(backAngle);

		if(enableLogging){ROS_INFO("leg mapping calculated angles for leg [%i] \t front = [%f] \t rear = [%f]", i, frontAngle, backAngle); };
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
