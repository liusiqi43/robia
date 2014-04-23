#ifndef ARDRONE_H__
#define ARDRONE_H__

#include <boost/utility.hpp>
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "camera.h"

class ARDrone : boost::noncopyable {
public:
	static const double TWIST_LINEAR;
	static const double TWIST_ANGULAR;

	static const int FRONT_CAM; 
	static const int BOTTOM_CAM; 

	ARDrone();
	~ARDrone();
	void takeOff();
	void land();
	void reset();

	void moveForward();
	void moveBackward();
	void moveLeft();
	void moveRight();
	void moveUp();
	void moveDown();

	void hover();

	void rotateLeft();
	void rotateRight();

private:
	//NodeHandle qui identifie le Node
	ros::NodeHandle n;

	ros::Publisher pubTakeOff;
	ros::Publisher pubLand;
	ros::Publisher pubReset;
	ros::Publisher pubMove;

	Camera *front_cam;
};

#endif /* end of include guard: ARDRONE_H__ */
