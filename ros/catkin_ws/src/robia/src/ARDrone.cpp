#include <geometry_msgs/Twist.h>
#include "ros/ros.h"
#include "std_msgs/Empty.h"

#include "ARDrone.h"
#include "camera.h"

const double ARDrone::TWIST_LINEAR = 0.3;
const double ARDrone::TWIST_ANGULAR = 0.3;
const int ARDrone::FRONT_CAM = 0;
const int ARDrone::BOTTOM_CAM = 1;


ARDrone::ARDrone() {
	// keep queue size to 1
    this->pubTakeOff = this->n.advertise<std_msgs::Empty>("/ardrone/takeoff", 0);
    this->pubLand = this->n.advertise<std_msgs::Empty>("/ardrone/land", 0);
    this->pubReset = this->n.advertise<std_msgs::Empty>("/ardrone/reset", 0);
    this->pubMove = this->n.advertise<geometry_msgs::Twist>("/cmd_vel", 0);

    this->front_cam = new Camera("/robia/gngt_output");
}

ARDrone::~ARDrone() {
    delete this->front_cam;
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

void ARDrone::moveForward() {
	moveForward(TWIST_LINEAR);   
}

void ARDrone::moveForward(double vel) {
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = vel;
    this->pubMove.publish(cmd_msg);
    ROS_INFO("Moving forward at %f", vel);      
}

void ARDrone::moveBackward() {
	 moveBackward(TWIST_LINEAR);
}

void ARDrone::moveBackward(double vel) {
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = -vel;
    this->pubMove.publish(cmd_msg);
    ROS_INFO("Moving backward");           
}

void ARDrone::moveLeft() {
    moveLeft(TWIST_LINEAR);      
}

void ARDrone::moveLeft(double vel) {
	geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.y = vel;
    this->pubMove.publish(cmd_msg);
    ROS_INFO("Moving left");      
}

void ARDrone::moveRight() {
    moveRight(TWIST_LINEAR);
}

void ARDrone::moveRight(double vel) {
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.y = -vel;
    this->pubMove.publish(cmd_msg);
    ROS_INFO("Moving right");      
}


void ARDrone::moveUp() {
    moveUp(TWIST_LINEAR);
}

void ARDrone::moveUp(double vel) {
	geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.z = vel;
    this->pubMove.publish(cmd_msg);
    ROS_INFO("Moving up");      
}

void ARDrone::moveDown() {
    moveDown(TWIST_LINEAR);
}

void ARDrone::moveDown(double vel) {
	geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.z = -vel;
    this->pubMove.publish(cmd_msg);
    ROS_INFO("Moving down");      
}


void ARDrone::rotateLeft() {
    rotateLeft(TWIST_LINEAR);
}

void ARDrone::rotateRight() {
    rotateRight(TWIST_LINEAR);
}

void ARDrone::rotateLeft(double vel) {
	geometry_msgs::Twist cmd_msg;
    cmd_msg.angular.z = -vel;
    this->pubMove.publish(cmd_msg);
    ROS_INFO("Rotate left");
}

void ARDrone::rotateRight(double vel) {
	geometry_msgs::Twist cmd_msg;
    cmd_msg.angular.z = vel;
    this->pubMove.publish(cmd_msg);
    ROS_INFO("Rotate right");
}

void ARDrone::hover() {
	geometry_msgs::Twist cmd_msg;
	cmd_msg.angular.x = 0.0;
	cmd_msg.angular.y = 0.0;

    ROS_INFO("Hover");
    this->pubMove.publish(cmd_msg);
}
