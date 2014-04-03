#include <boost/utility.hpp>
#include "ros/ros.h"
#include "std_msgs/Empty.h"



class ARDrone : boost::noncopyable {
public:
	const double TWIST_LINEAR;
	const double TWIST_ANGULAR;

	ARDrone();
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

	ros::NodeHandle n;

	ros::Publisher pubTakeOff;
	ros::Publisher pubLand;
	ros::Publisher pubReset;
	ros::Publisher pubMove;
};
