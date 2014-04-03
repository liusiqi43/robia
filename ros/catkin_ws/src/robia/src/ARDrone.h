#include <boost/utility.hpp>
#include "ros/ros.h"
#include "std_msgs/Empty.h"



class ARDrone : boost::noncopyable {
public:
	enum DroneState{
		LANDED, 
		FLYING
	};

	ARDrone();
	void takeOff();
	void land();
	void reset();
private:
	DroneState state;

	ros::NodeHandle n;

	ros::Publisher pubTakeOff;
	ros::Publisher pubLand;
	ros::Publisher pubReset;

};
