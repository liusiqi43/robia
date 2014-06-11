#include <ros/ros.h>
#include "control.h"

#define depth_control_activation_epsilon 1e-1 // 1e-1
#define rotate_epsilon 1e-1
#define height_epsilon 1e-2
#define depth_epsilon 1e-3
#define translate_epsilon 1e-2
#define depthPosition 3e-1


Control::Control() : idleCount(0) {
	drone = new ARDrone();

	ros::NodeHandle nc;
  	sub = nc.subscribe("/robia/output_positions", 1, &Control::callback, this);

}

Control::~Control() {
	delete drone;
}


void Control::centerWithRespectToBaryCenter() {
	//calcul des coordonnées du barycentre principal

	BARx = (1/3.)*(Headx + Rightx + Leftx);
	BARy = (1/3.)*(Heady + Righty + Lefty);


	ROS_INFO("(BarX, BarY): (%f, %f)", BARx, BARy);

	//instructions à envoyer suivant les cas
/*	if(BARx<0.5-epsilon)
	{
		drone->setMoveLeft();
	}
	if(BARy<0.5-epsilon)
	{
		drone->setMoveUp();
	}
	if(BARx>0.5+epsilon)
	{
		drone->setMoveRight();
	}
	if(BARy>0.5+epsilon)
	{
		drone->setMoveDown();
	}*/

	// ROS_INFO("abs: %f : epsilon : %f", std::abs(BARx - 0.5), epsilon);

 	// ROS_INFO("SET MOVELEFT: %f", tan((0.5 - BARx)/1.5));
	if(BARx - 0.5 > rotate_epsilon) 
		drone->setRotateLeft();
	else if (BARx - 0.5 < -rotate_epsilon) 
		drone->setRotateRight();
	else drone->setRotateRight(0);

	// ROS_INFO("SET MOVEUP: %f", tan((0.5 - BARy)/1.5));
	// tan((0.5 - BARy)/1.5)
	if(BARy - 0.5 > height_epsilon) drone->setMoveDown();
	else if (BARy - 0.5 < -height_epsilon) drone->setMoveUp();
	else drone->setMoveUp(0);

}

void Control::checkForEqualDistancesBetweenLeftAndRight() {

	//actions à effectuer

	if(d1/(d1 + d2) > 0.5 + translate_epsilon)
	{
		drone->setMoveLeft();
	}
	else if(d1/(d1 + d2) < 0.5 - translate_epsilon)
	{
		drone->setMoveRight();
	} else {
		drone->setMoveLeft(0);
	}
}

void Control::controlOfDepth() {

	// {
	// if(std::abs((d1+d2) - depthPosition) < epsilon && std::abs(Headx - (Leftx + Rightx)/2) < epsilon)
	// 	drone->setMoveForward(tan((depthPosition - (d1 + d2))/1.5));
	// }
	// if((d1+d2)>depthPosition+epsilon && abs(Headx - (Leftx + Rightx)/2) < epsilon)
	// {
	// 	drone->setMoveBackward();
	// }


	if (abs(Headx - (Leftx + Rightx)/2) < depth_control_activation_epsilon)
	{
		if((d1+d2)>depthPosition+depth_epsilon)
			drone->setMoveBackward();
		else if ((d1+d2)<depthPosition-depth_epsilon)
			drone->setMoveForward();
		else 
			drone->setMoveForward(0);
	}

}

#define COUNT_TILL_HOVER 50

void Control::callback(const robia::points_tuple &msg) {

	if(msg.Hx == -1) {
		idleCount++;
		if (idleCount >= COUNT_TILL_HOVER) {
			idleCount = 0;
			drone->hover();
			drone->commit();
		}
		return;
	}

	idleCount = 0;
	//on reçoit le message du topic output_positions
	Headx = msg.Hx;//à modifier suivant le nom des floats du message
	Heady = msg.Hy;
	Rightx = msg.Rx; // Rx < Hx < Lx sur l'image
	Righty = msg.Ry;
	Leftx = msg.Lx;
	Lefty = msg.Ly;

	//calcul des distances
	d1 = Headx - Rightx;
	d2 = Leftx - Headx;

	ROS_INFO("d1: %f, d2: %f", d1, d2);

	centerWithRespectToBaryCenter();
	checkForEqualDistancesBetweenLeftAndRight();
	controlOfDepth();


	drone->commit();
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "Control_Node");

  Control control;

  ros::spin();


  return 0;
}