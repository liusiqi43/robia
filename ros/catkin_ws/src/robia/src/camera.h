#ifndef CAMERA_H__
#define CAMERA_H__

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace cv_bridge;

class Camera
{
private:
	const std::string subscribedTopic;
	bool shouldDisplay;
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub;

	void imageCallBack(const sensor_msgs::ImageConstPtr& original_image);
public:
	Camera(const std::string& topic);
	void toggleDisplay() { this->shouldDisplay = !shouldDisplay; ROS_INFO("Display: %d", this->shouldDisplay);}
};


#endif /* end of include guard: CAMERA_H__ */
