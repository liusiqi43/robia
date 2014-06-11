#include <geometry_msgs/Twist.h>
#include "ros/ros.h"
#include "std_msgs/Empty.h"

#include "ARDrone.h"
#include "camera.h"

#include <array>

const double ARDrone::TWIST_TRANSLATION_LINEAR = 0.02;
const double ARDrone::TWIST_DEPTH_LINEAR = 0.04;
const double ARDrone::TWIST_HEIGHT_LINEAR = 0.05;
const double ARDrone::TWIST_ANGULAR = 0.15;
const int ARDrone::FRONT_CAM = 0;
const int ARDrone::BOTTOM_CAM = 1;

ARDrone::ARDrone() : needCommit(false) {
	// keep queue size to 1
    this->pubTakeOff = this->n.advertise<std_msgs::Empty>("/ardrone/takeoff", 0);
    this->pubLand = this->n.advertise<std_msgs::Empty>("/ardrone/land", 0);
    this->pubReset = this->n.advertise<std_msgs::Empty>("/ardrone/reset", 0);
    this->pubMove = this->n.advertise<geometry_msgs::Twist>("/cmd_vel", 0);

    // this->front_cam = new Camera("/robia/gngt_output");
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

void ARDrone::setMoveForward() {
	setMoveForward(TWIST_DEPTH_LINEAR);
}

void ARDrone::setMoveForward(double vel) {
    cmd_msg.linear.x = vel;
    needCommit = true;
}

void ARDrone::setMoveBackward() {
	setMoveBackward(TWIST_DEPTH_LINEAR);
}

void ARDrone::setMoveBackward(double vel) {
    cmd_msg.linear.x = -vel;  
    needCommit = true;
}

void ARDrone::setMoveLeft() {
    setMoveLeft(TWIST_TRANSLATION_LINEAR);      
}

void ARDrone::setMoveLeft(double vel) {
    cmd_msg.linear.y = vel;
    needCommit = true;
}

void ARDrone::setMoveRight() {
    setMoveRight(TWIST_TRANSLATION_LINEAR);
}

void ARDrone::setMoveRight(double vel) {
    cmd_msg.linear.y = -vel;
    needCommit = true;
}

void ARDrone::setMoveUp() {
    setMoveUp(TWIST_HEIGHT_LINEAR);
}

void ARDrone::setMoveUp(double vel) {
    cmd_msg.linear.z = vel;
    needCommit = true;
}

void ARDrone::setMoveDown() {
    setMoveDown(TWIST_HEIGHT_LINEAR);
}

void ARDrone::setMoveDown(double vel) {
    cmd_msg.linear.z = -vel;
    needCommit = true;
}

void ARDrone::setRotateLeft() {
    setRotateLeft(TWIST_ANGULAR);
}

void ARDrone::setRotateRight() {
    setRotateRight(TWIST_ANGULAR);
}

void ARDrone::setRotateLeft(double vel) {
    cmd_msg.angular.z = -vel;
    needCommit = true;
}

void ARDrone::setRotateRight(double vel) {
    cmd_msg.angular.z = vel;
    needCommit = true;
}

void ARDrone::hover() {
    cmd_msg.linear.x = 0.0;
    cmd_msg.linear.y = 0.0;
    cmd_msg.linear.z = 0.0;
    cmd_msg.angular.x = 0.0;
	cmd_msg.angular.y = 0.0;
	cmd_msg.angular.z = 0.0;

    needCommit = true;
}

#define epsilon 1e-3
void ARDrone::commit() {
    if (needCommit) {

        // -linear.x: move backward
        // +linear.x: move forward
        // -linear.y: move right
        // +linear.y: move left
        // -linear.z: move down
        // +linear.z: move up

        // -angular.z: turn left
        // +angular.z: turn right


        char a = '-', t = '-', h = '-', r = '-';
        if (cmd_msg.linear.x < -epsilon)
            a = 'r';
        else if (cmd_msg.linear.x > epsilon)
            a = 'a';


        if (cmd_msg.linear.y < -epsilon)
            t = 'd';
        else if (cmd_msg.linear.y > epsilon)
            t = 'g';

        if (cmd_msg.linear.z < -epsilon)
            h = 'b';
        else if (cmd_msg.linear.z > epsilon)
            h = 'h';

        if (cmd_msg.angular.z < -epsilon)
            r = 'g';
        else if (cmd_msg.angular.z > epsilon)
            r = 'd';

        std::array<char, 5> res = {{ a, t, h, r, 0 }};



        // ROS_INFO("===============\nlinear.x = %f\nlinear.y = %f\nlinear.z = %f\nangular.x = %f\nangular.y = %f\nangular.z = %f\n", cmd_msg.linear.x, cmd_msg.linear.y, cmd_msg.linear.z, cmd_msg.angular.x, cmd_msg.angular.y, cmd_msg.angular.z);
        ROS_INFO("%s", res.begin());
        this->pubMove.publish(cmd_msg);
        
        
        geometry_msgs::Twist temp;
        cmd_msg = temp;
        needCommit = false;
    }
}
