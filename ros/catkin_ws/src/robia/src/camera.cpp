#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "camera.h"

Camera::Camera(const std::string& topic) : subscribedTopic(topic), shouldDisplay(true) {
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    sub = it.subscribe(this->subscribedTopic.c_str(), 1, &Camera::imageCallBack, this);
    ROS_INFO("Camera subscribe to %s constructed\n", this->subscribedTopic.c_str());
    // cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    cvNamedWindow(this->subscribedTopic.c_str());
}

Camera::~Camera() {
    cv::destroyWindow(this->subscribedTopic.c_str());
}

void Camera::imageCallBack(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    

    // Update GUI Window
    cv::imshow(this->subscribedTopic.c_str(), cv_ptr->image);
    cv::waitKey(3);
}