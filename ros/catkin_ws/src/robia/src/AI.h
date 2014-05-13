#include "dyeFilter.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <vq2/vq2.h>
#include <opencv2/opencv.hpp>

// class AI;

// class AI::Params;
// class AI::Similarity;
// class AI::UnitSimilarity;
// class AI::Learn;
// class AI::UnitLearn;
// class AI::Evolution;

typedef vq2::algo::gngt::Unit<cv::Point2d>                Unit;
typedef vq2::Graph<Unit,char,Unit::copy_constructor>    Graph;
typedef Graph::vertex_type                              Vertex;
typedef Graph::edge_type                                Edge;
typedef Graph::ref_vertex_type                          RefVertex;
typedef Graph::ref_edge_type                            RefEdge;

class AI
{

public:
	const int DESIRED_SAMPLE_SIZE;
	const int NB_EPOCHS_PER_FRAME;

	AI();
	~AI();

	void imageCallBack(const sensor_msgs::ImageConstPtr& msg);

	class Similarity {
	public:
		typedef cv::Point2d value_type;
		typedef cv::Point2d sample_type;

		double operator()(const value_type& arg1,
			const sample_type& arg2) {
			double dx = arg1.x - arg2.x;
			double dy = arg1.y - arg2.y;
			return dx*dx + dy*dy;
		}
	};
	typedef vq2::unit::Similarity<Unit,Similarity> UnitSimilarity;

	// Comprend pas. ?????????????????????? coef sert a quoi?
	class Learn {
	public:
		typedef cv::Point2d sample_type;
		typedef cv::Point2d weight_type;
		void operator()(double coef,
			weight_type& prototype,
			const sample_type& target) {
			prototype.x += coef * (target.x - prototype.x);
			prototype.y += coef * (target.y - prototype.y);
		}
	};
	typedef vq2::unit::Learn<Unit,Learn> UnitLearn;

	#define TARGET      2e-4
	class Params {
	public:
	  int nb_samples;

	  // GNG-T
		int ageMax(void)           {return 20;}
		double learningRate(void)  {return .001;}
		double learningRatio(void) {return .2;}
		double lambda(void)        {return .001;}
		double infinity(void)      {return 1e12;}

	  // Evolution
		double target(void)        {return TARGET;}
		int nbSamples(void)        {return nb_samples;}
		double lowPassCoef(void)   {return .4;}
		double delta(void)         {return .75;}
		double margin(void)        {return .2;}
	}; 
	typedef vq2::by_default::gngt::Evolution<Params> Evolution;

	const cv::Point2d& sample_of(const cv::Point2d& p) {return p;}



	// functor
	class DisplayEdge {
		cv::Mat& rImage;
	public:
		DisplayEdge(cv::Mat &img) : rImage(img) {}

		bool operator()(Edge& e) {
			cv::Point2d A = (*(e.n1)).value.prototype();
			cv::Point2d B = (*(e.n2)).value.prototype();

			cv::line(rImage, A, B, CV_RGB(255, 255, 255));

	    return false; // the element should not be removed.
	}
};

	// This is a loop functor class.
class DisplayVertex {
	   cv::Mat&  rImage; // Référence sur l'image opencv
	public:
		DisplayVertex(cv::Mat &img) : rImage(img) {}

		bool operator()(Vertex& n) { 
			cv::Point2d A = n.value.prototype();

			cv::circle(rImage, A, 3, CV_RGB(255, 0, 0));

	    return false; // The element should not be removed.
	}
};

private:
	GR::DyeFilter *dyeFilter;
	image_transport::Publisher pubImageFiltrer;
	image_transport::Subscriber sub;
	Params           params;
	Similarity       distance;
	UnitSimilarity   unit_distance;
	Learn            learn;
	UnitLearn        unit_learn;
	Evolution        evolution;
	std::vector<cv::Point2d> gngt_input;
	Graph		     graph;

	cv::VideoWriter mOutVideo;
};
