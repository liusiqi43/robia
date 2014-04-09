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

    this->front_cam = new Camera("/ardrone/front/image_raw");
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
	geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = TWIST_LINEAR;
    this->pubMove.publish(cmd_msg);
    ROS_INFO("Moving forward");      
}

void ARDrone::moveBackward() {
	geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = -TWIST_LINEAR;
    this->pubMove.publish(cmd_msg);
    ROS_INFO("Moving backward");      
}

void ARDrone::moveLeft() {
	geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.y = TWIST_LINEAR;
    this->pubMove.publish(cmd_msg);
    ROS_INFO("Moving left");      
}

void ARDrone::moveRight() {
	geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.y = -TWIST_LINEAR;
    this->pubMove.publish(cmd_msg);
    ROS_INFO("Moving right");      
}

void ARDrone::moveUp() {
	geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.z = TWIST_LINEAR;
    this->pubMove.publish(cmd_msg);
    ROS_INFO("Moving up");      
}

void ARDrone::moveDown() {
	geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.z = -TWIST_LINEAR;
    this->pubMove.publish(cmd_msg);
    ROS_INFO("Moving down");      
}

void ARDrone::rotateLeft() {
	geometry_msgs::Twist cmd_msg;
    cmd_msg.angular.z = -TWIST_LINEAR;
    this->pubMove.publish(cmd_msg);
    ROS_INFO("Rotate left");
}

void ARDrone::rotateRight() {
	geometry_msgs::Twist cmd_msg;
    cmd_msg.angular.z = TWIST_LINEAR;
    this->pubMove.publish(cmd_msg);
    ROS_INFO("Rotate right");
}

void ARDrone::hover() {
	geometry_msgs::Twist cmd_msg;
	cmd_msg.angular.x = 0.0;
	cmd_msg.angular.y = 0.0;

    this->pubMove.publish(cmd_msg);
}

void ARDrone::toggleCamDisplay(int index) {
    switch (index) {
        case FRONT_CAM:
            this->front_cam->toggleDisplay();
            break;
        case BOTTOM_CAM:
            ROS_INFO("NO BOTTOM_CAM");
            break;
    }
}