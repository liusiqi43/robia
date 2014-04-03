#include "ARDrone.h"
#include "ros/ros.h"
#include "std_msgs/Empty.h"

ARDrone::ARDrone() : state(LANDED) {
	// keep queue size to 1
    this->pubTakeOff = this->n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    this->pubLand = this->n.advertise<std_msgs::Empty>("/ardrone/land", 1);
    this->pubReset = this->n.advertise<std_msgs::Empty>("/ardrone/reset", 1);
    this->subState = this->n.subscribe("/ardrone/navdata", 1000, chatterCallback);
}

void ARDrone::takeOff() {
	this->pubTakeOff.publish(std_msgs::Empty());
	ROS_INFO("taking off...");
}

void ARDrone::land() {
	this->pubLand.publish(std_msgs::Empty());
	ROS_INFO("Landing...");
}

void ARDrone::reset() {
	this->pubReset.publish(std_msgs::Empty());
	ROS_INFO("Reset");
}

int getState() {

}