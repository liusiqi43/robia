#include "ros/ros.h"
#include "dyeFilter.h"
#include "AI.h"
#include <image_transport/image_transport.h>


#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

AI::AI() {
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    dyeFilter = new GR::DyeFilter(1., 0.5, 50);
    pubImageFiltrer = it.advertise("robia/colorImageFilter", 50);

    image_transport::Subscriber sub = it.subscribe("/ardrone/front/image_raw", 1, &AI::imageCallBack, this);
}

AI::~AI() {
    delete dyeFilter;
}


void AI::imageCallBack(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    std::vector<AI::Point> gngt_input;

    GR::DyeFilter *dyeFilter = new GR::DyeFilter(1., 0.5, 50);
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  // Processing



  dyeFilter->process(cv_ptr->image, cv_ptr->image, gngt_input);



  // Processing ends here

    // Publish processed image
  pubImageFiltrer.publish(cv_ptr->toImageMsg());
  cv::waitKey(3);
}




// int main(int argc, char *argv[])
// {
// 	ros::init(argc, argv, "colorFilterNode");

// 	ros::NodeHandle n;
//     image_transport::ImageTransport it(n);

//     pubImageFiltrer = it.advertise("robia/colorImageFilter", 50);

//     image_transport::Subscriber sub = it.subscribe("/ardrone/front/image_raw", 1, &imageCallBack);

//     ros::spin();

//     return 0;
// }