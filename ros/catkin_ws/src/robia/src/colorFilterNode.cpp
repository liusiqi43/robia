#include "dyeFilter.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


image_transport::Publisher pubImageFiltrer;


void imageCallBack(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Publish processed image
    pubImageFiltrer.publish(cv_ptr->toImageMsg());
    cv::waitKey(3);
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "colorFilterNode");

	ros::NodeHandle n;
    image_transport::ImageTransport it(n);

  pubImageFiltrer = it.advertise("robia/colorImageFilter", 50);

    image_transport::Subscriber sub = it.subscribe("/ardrone/front/image_raw", 1, &imageCallBack);

    ros::spin();

	return 0;
}