#include "dyeFilter.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


image_transport::Publisher pubImageFiltrer;

// Renvoyer l'image filtré
void imageCallBack(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;

    GR::DyeFilter *dyeFilter = new GR::DyeFilter(1., 0.5, 50);
    try {
      // Transformer opencv image à ROS image
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // cv::Mat output;
    dyeFilter->process(cv_ptr->image, cv_ptr->image);
    delete dyeFilter;


    // try {
    //   cv_ptr_output = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    // } catch (cv_bridge::Exception& e) {
    //   ROS_ERROR("cv_bridge exception: %s", e.what());
    //   return;
    // }

    // Publish processed image
    pubImageFiltrer.publish(cv_ptr->toImageMsg());
    cv::waitKey(3);
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "colorFilterNode");

	ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    // Publier au maximum 50 messages vers le node
    pubImageFiltrer = it.advertise("robia/colorImageFilter", 50);

    // Abonné l'un le plus précédent  
    image_transport::Subscriber sub = it.subscribe("/ardrone/front/image_raw", 1, &imageCallBack);

    // Pour ne pas terminer le programme
    ros::spin();

	return 0;
}