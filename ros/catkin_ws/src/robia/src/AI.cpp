#include "ros/ros.h"
#include "dyeFilter.h"
#include "AI.h"

#include <opencv2/opencv.hpp> // cv::Point2d, cv::
#include <algorithm> // random_shuffle
#include <array>

#include <image_transport/image_transport.h> // image_transport
#include <cv_bridge/cv_bridge.h> // ROS>>>CV cv>>ROS
#include <sensor_msgs/image_encodings.h> // ROS>>>CV cv>>ROS
#include <opencv2/imgproc/imgproc.hpp> // ROS>>>CV cv>>ROS
#include <opencv2/highgui/highgui.hpp> // ROS>>>CV cv>>ROS

AI::AI() : DESIRED_SAMPLE_SIZE(100), 
NB_EPOCHS_PER_FRAME(10), 
unit_distance(distance),
unit_learn(learn),
evolution(params),
gngt_input(), 
DEBUG(true)
{

  dyeFilter = new GR::DyeFilter(1., 0.5, 50);

  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  pubImageFiltrer = it.advertise("/robia/gngt_output", 50);
  sub = it.subscribe("/ardrone/front/image_raw", 1, &AI::imageCallBack, this);
  ROS_INFO("AI constructed");


  //Set properties  
 int askFileTypeBox=1; //-1 is show box of codec  
 int Color = 1;  
 cv::Size S = cv::Size( 320, 240 );  

 //make output video file  
 if (DEBUG) mOutVideo.open("../output.avi", CV_FOURCC('P','I','M','1'), 24, S, Color);  
}

AI::~AI() {
  delete dyeFilter;
}


void AI::imageCallBack(const sensor_msgs::ImageConstPtr& msg) {
  // ROS_INFO("AI::imageCallBack triggered");
  cv_bridge::CvImagePtr cv_ptr;


  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    // Afficher les détailles des erreurs
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Processing



  cv::Size imageSize =  cv_ptr->image.size();
  dyeFilter->process(cv_ptr->image, cv_ptr->image, gngt_input);

  for(int e = 0; e < NB_EPOCHS_PER_FRAME; ++e) {
    std::random_shuffle(gngt_input.begin(),gngt_input.end()); // Important !!!!
    auto begin = gngt_input.begin();
    int vecsize = gngt_input.size()-1;

    int sampleSize = std::min(DESIRED_SAMPLE_SIZE, vecsize);
    int nbSample = imageSize.height * imageSize.width * sampleSize / vecsize;
    params.setNbSamples(nbSample);

    auto end   = gngt_input.begin()+sampleSize; 

    vq2::algo::gngt::open_epoch(graph,evolution);
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

  // Composantes connexes
  std::map<unsigned int,Graph::Component*> components;
  graph.computeConnectedComponents(components,false);

  for(auto iter = components.begin();iter != components.end();++iter) {
    std::cout << "Label " << (*iter).first << std::endl;
    auto comp = (*iter).second;
    ComputeGravity compute_g(cv_ptr->image);
    comp->for_each_vertex(compute_g);
    compute_g.drawBaryCenter();
  }


  // Processing ends here

  // Publish processed image

  pubImageFiltrer.publish(cv_ptr->toImageMsg());

  if (DEBUG) mOutVideo << cv_ptr->image;
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
