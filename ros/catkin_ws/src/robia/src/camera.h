#ifndef CAMERA_H__
#define CAMERA_H__

#include <ros/ros.h>
#include <image_transport/image_transport.h>

class Camera
{
private:
	const std::string subscribedTopic;
	bool shouldDisplay;
    image_transport::Subscriber sub;

	void imageCallBack(const sensor_msgs::ImageConstPtr& original_image);
public:
	Camera(const std::string& topic);
	~Camera();
	void toggleDisplay() { this->shouldDisplay = !shouldDisplay; ROS_INFO("Display: %d", this->shouldDisplay);}
};


#endif /* end of include guard: CAMERA_H__ */
