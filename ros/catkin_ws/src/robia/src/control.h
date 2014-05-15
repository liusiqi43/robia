#pragma once

#include <ros/ros.h>
#include <robia/points_tuple.h>
#include "ARDrone.h"


class Control {
public:

Control();
~Control();
void centerWithRespectToBaryCenter();
void checkForEqualDistancesBetweenLeftAndRight();
void controlOfDepth();
void callback(const robia::points_tuple &msg);

private:
	ARDrone *drone;

	ros::Subscriber sub;

	ros::NodeHandle nc;
	double Headx;
	double Heady;
	double Rightx;
	double Righty;
	double Leftx;
	double Lefty;

	double BARx;
	double BARy;

	double d1;
	double d2;
};
