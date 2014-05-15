#include "ros/ros.h"
#include "dyeFilter.h"
#include "AI.h"
#include <robia/points_tuple.h>

#include <opencv2/opencv.hpp> // cv::Point2d, cv::
#include <algorithm> // random_shuffle
#include <array>

#include <image_transport/image_transport.h> // image_transport
#include <cv_bridge/cv_bridge.h> // ROS>>>CV cv>>ROS
#include <sensor_msgs/image_encodings.h> // ROS>>>CV cv>>ROS
#include <opencv2/imgproc/imgproc.hpp> // ROS>>>CV cv>>ROS
#include <opencv2/highgui/highgui.hpp> // ROS>>>CV cv>>ROS


#define LengthOfCamera 320
#define HeightOfCamera 240
#define MIN_VALID_COMPONENT_DEVIATION 3


AI::AI() : DESIRED_SAMPLE_SIZE(2000), 
NB_EPOCHS_PER_FRAME(10), 
unit_distance(distance),
unit_learn(learn),
evolution(params),
gngt_input(), 
labelToColor(),
DEBUG(true),
rng(12345)
{

  dyeFilter = new GR::DyeFilter(1., 0.5, 50);

  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  pubImageFiltrer = it.advertise("/robia/gngt_output", 50);
  pubThreePointsPositions = n.advertise<robia::points_tuple>("/robia/output_positions", 50);

  //on publie les messages contenant les coordonnées des 3 points sur un topic 


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

  // First, we invalidate long edges. 
  InvalidateLongEdge invalidate_long_edge;
  graph.for_each_edge(invalidate_long_edge);
  // Second, we invalidate any vertex that owns only invalid edges.
  InvalidateNoisyVertex invalid_noisy_vertex;
  graph.for_each_vertex(invalid_noisy_vertex);

  // Composantes connexes
  std::map<unsigned int, Graph::Component*> components;
  graph.computeConnectedComponents(components, true);
  // Create pair <Deviation,Point,CompareWithDeviation>
  std::priority_queue< std::pair<double, cv::Point2d>, 
                          std::vector<std::pair<double, cv::Point2d> >, 
                          CompareComponentsWithDeviation> largestThreeComponentQueue;

  for(auto iter = components.begin();iter != components.end();++iter) {
    VertexLooper vertexLooper(cv_ptr->image);
    EdgeLooper edgeLooper(cv_ptr->image);

    if (labelToColor.find((*iter).first) == labelToColor.end()) {
      // new label
      labelToColor.insert(std::pair<unsigned int, cv::Scalar>((*iter).first, 
        cv::Scalar(rng.uniform(0,255), 
          rng.uniform(0, 255), 
          rng.uniform(0, 255))));
    }

    cv::Scalar color = labelToColor.find(iter->first)->second;
    edgeLooper.setEdgeColor(color);
    vertexLooper.setVertexColor(color);

    auto comp = (*iter).second;

    comp->for_each_vertex(vertexLooper);
    comp->for_each_edge(edgeLooper);

    vertexLooper.drawBaryCenter();
    vertexLooper.getDeviation();

    if (vertexLooper.getDeviation() >= MIN_VALID_COMPONENT_DEVIATION) {
      if (largestThreeComponentQueue.size() < 3) {
        largestThreeComponentQueue.push(
                std::make_pair(
                  vertexLooper.getDeviation(), 
                  vertexLooper.getBaryCenter()
                  )
                );
      } else {
        if(largestThreeComponentQueue.top().first < vertexLooper.getDeviation()) {
          largestThreeComponentQueue.pop();
          largestThreeComponentQueue.push(
                std::make_pair(
                  vertexLooper.getDeviation(), 
                  vertexLooper.getBaryCenter()
                  )
                );
        }
      }
    } 

    //if (DEBUG) ROS_INFO("Label %d, Deviation %f", (*iter).first, vertexLooper.getDeviation());

  }

  // Processing ends here

  // Publish barycenters for three major components
  if (largestThreeComponentQueue.size() == 3)
    publishThreeComponentRHL(largestThreeComponentQueue);
  // Publish processed image
  pubImageFiltrer.publish(cv_ptr->toImageMsg());

  if (DEBUG) mOutVideo << cv_ptr->image;
  // ROS_INFO("AI::imageCallBack published imageMsg");
  cv::waitKey(3);
}

bool comparePointsWithRespectToX (const cv::Point2d &a, const cv::Point2d &b) { return (a.x < b.x); }

void AI::publishThreeComponentRHL(std::priority_queue < std::pair<double, cv::Point2d>, 
    std::vector<std::pair<double, cv::Point2d> >, 
    AI::CompareComponentsWithDeviation> largestThreeComponentQueue) {

    std::vector<cv::Point2d> largestThreeComponentVector;

    while(!largestThreeComponentQueue.empty()) {
      largestThreeComponentVector.push_back(largestThreeComponentQueue.top().second);
      largestThreeComponentQueue.pop();
    }

    std::sort(largestThreeComponentVector.begin(), largestThreeComponentVector.end(), comparePointsWithRespectToX);

    //create message
    robia::points_tuple message;

    int s = largestThreeComponentVector.size();

    // Left Hand, Head, Right hand
    message.Rx = largestThreeComponentVector[0].x/LengthOfCamera;
    message.Ry = largestThreeComponentVector[0].y/HeightOfCamera;
    message.Hx = largestThreeComponentVector[1].x/LengthOfCamera;
    message.Hy = largestThreeComponentVector[1].y/HeightOfCamera;
    message.Lx = largestThreeComponentVector[2].x/LengthOfCamera;
    message.Ly = largestThreeComponentVector[2].y/HeightOfCamera;

    // Publish three points to ROSTopic 
    pubThreePointsPositions.publish(message);

  }


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "AI_Node");

  ros::NodeHandle n;

  AI ai;

  ros::spin();

  return 0;
}
