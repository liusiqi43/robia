#include "ros/ros.h"
#include "dyeFilter.h"
#include "AI.h"

#include <opencv2/opencv.hpp> // cv::Point2d, cv::
#include <algorithm> // random_shuffle

#include <image_transport/image_transport.h> // image_transport
#include <cv_bridge/cv_bridge.h> // ROS>>>CV cv>>ROS
#include <sensor_msgs/image_encodings.h> // ROS>>>CV cv>>ROS
#include <opencv2/imgproc/imgproc.hpp> // ROS>>>CV cv>>ROS
#include <opencv2/highgui/highgui.hpp> // ROS>>>CV cv>>ROS

AI::AI() : DESIRED_SAMPLE_SIZE(100), 
          NB_EPOCHS_PER_FRAME(10), 
          unit_distance(distance),
          unit_learn(learn),
          evolution(params)
           {

    dyeFilter = new GR::DyeFilter(1., 0.5, 50);

    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    pubImageFiltrer = it.advertise("/robia/gngt_output", 50);
    sub = it.subscribe("/ardrone/front/image_raw", 1, &AI::imageCallBack, this);
    ROS_INFO("AI constructed");
}

AI::~AI() {
    delete dyeFilter;
}


void AI::imageCallBack(const sensor_msgs::ImageConstPtr& msg) {
  // ROS_INFO("AI::imageCallBack triggered");
    cv_bridge::CvImagePtr cv_ptr;

    /**
    GNGT
    **/
    std::vector<cv::Point2d> gngt_input;

    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

  // Processing



    dyeFilter->process(cv_ptr->image, cv_ptr->image, gngt_input);

    for(int e = 0; e < NB_EPOCHS_PER_FRAME; ++e) {
      std::random_shuffle(gngt_input.begin(),gngt_input.end()); // Important !!!!
      auto begin = gngt_input.begin();
      int vecsize = gngt_input.size()-1;
      int sampleSize = std::min(DESIRED_SAMPLE_SIZE, vecsize);
      auto end   = gngt_input.begin()+sampleSize; 

      for(auto iter = begin; iter != end; ++iter)
        vq2::algo::gngt::submit(params,graph,
          unit_distance,unit_learn,
          *iter,true);
      vq2::algo::gngt::close_epoch(params,graph,
       unit_learn,
       evolution,true);
    }


    DisplayVertex display_v(cv_ptr->image);
    DisplayEdge display_e(cv_ptr->image);


    graph.for_each_vertex(display_v);
    graph.for_each_edge(display_e);

  // Processing ends here

    // Publish processed image
    pubImageFiltrer.publish(cv_ptr->toImageMsg());
     // ROS_INFO("AI::imageCallBack published imageMsg");
    cv::waitKey(3);
  }




int main(int argc, char *argv[])
{
	ros::init(argc, argv, "AI_Node");

  ros::NodeHandle n;

  AI ai;
	
  ros::spin();

  return 0;
}