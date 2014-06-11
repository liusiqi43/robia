#ifndef ARDRONE_H__
#define ARDRONE_H__

#include <boost/utility.hpp>
#include <geometry_msgs/Twist.h>
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "camera.h"

class ARDrone : boost::noncopyable {
public:
	static const double TWIST_TRANSLATION_LINEAR;
	static const double TWIST_DEPTH_LINEAR;
	static const double TWIST_HEIGHT_LINEAR;
	static const double TWIST_ANGULAR;

	static const int FRONT_CAM; 
	static const int BOTTOM_CAM; 

	ARDrone();
	~ARDrone();
	void takeOff();
	void land();
	void reset();

	void setMoveForward();
	void setMoveBackward();
	void setMoveLeft();
	void setMoveRight();
	void setMoveUp();
	void setMoveDown();

	void setMoveForward(double vel);
	void setMoveBackward(double vel);
	void setMoveLeft(double vel);
	void setMoveRight(double vel);
	void setMoveUp(double vel);
	void setMoveDown(double vel);

	void hover();

	void setRotateLeft(double vel);
	void setRotateRight(double vel);
	
	void setRotateLeft();
	void setRotateRight();

	void commit();

private:
	//NodeHandle qui identifie le Node
	ros::NodeHandle n;

	ros::Publisher pubTakeOff;
	ros::Publisher pubLand;
	ros::Publisher pubReset;
	ros::Publisher pubMove;

	geometry_msgs::Twist cmd_msg;

	Camera *front_cam;
	bool needCommit;
};

#endif /* end of include guard: ARDRONE_H__ */
