#include <ros/ros.h>
#include "control.h"


#define epsilon 0.1
#define depthPosition 0.3


Control::Control() {
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
	if(BARx<0.5-epsilon)
	{
		drone->moveLeft();
	}
	if(BARy<0.5-epsilon)
	{
		drone->moveUp();
	}
	if(BARx>0.5+epsilon)
	{
		drone->moveRight();
	}
	if(BARy>0.5+epsilon)
	{
		drone->moveDown();
	}
}

void Control::checkForEqualDistancesBetweenLeftAndRight() {

	//actions à effectuer

	if(d1>d2)
	{
		drone->moveLeft();
		drone->rotateRight();
	}
	if(d1<d2)
	{
		drone->moveRight();
		drone->rotateLeft();
	}

}

void Control::controlOfDepth() {

	if((d1+d2)<depthPosition-epsilon)
	{
		drone->moveForward();
	}
	if((d1+d2)>depthPosition+epsilon)
	{
		drone->moveBackward();
	}
}

void Control::callback(const robia::points_tuple &msg) {

	if(msg.Hx == -1) {
		drone->hover();
		return;
	}
	//on reçoit le message du topic output_positions
	Headx = msg.Hx;//à modifier suivant le nom des floats du message
	Heady = msg.Hy;
	Rightx = msg.Rx;
	Righty = msg.Ry;
	Leftx = msg.Lx;
	Lefty = msg.Ly;

	//calcul des distances
	d1 = Headx - Rightx;
	d2 = Leftx - Headx;


	centerWithRespectToBaryCenter();
	checkForEqualDistancesBetweenLeftAndRight();
	 controlOfDepth();
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "Control_Node");

  Control control;

  ros::spin();


  return 0;
}