#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "camera.h"

using namespace std;

Camera::Camera(const string& topic) : subscribedTopic(topic), shouldDisplay(false), it(nh) {
    image_transport::Subscriber sub = it.subscribe(this->subscribedTopic, 1, &Camera::imageCallBack, this);
}

void Camera::imageCallBack(const sensor_msgs::ImageConstPtr& original_image) {
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
    //Always copy, returning a mutable CvImage
    //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
    //if there is an error during conversion, display it
        ROS_ERROR("ARDrone.Camera Exception: %s", e.what());
        return;
    }
    if (shouldDisplay) {
        cv::namedWindow(subscribedTopic, CV_WINDOW_AUTOSIZE);
        cv::imshow(subscribedTopic, cv_ptr->image); 
    } else {
        cv::destroyWindow(subscribedTopic);
    }
    cv::waitKey(3);
}