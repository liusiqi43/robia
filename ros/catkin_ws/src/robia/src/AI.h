#include "dyeFilter.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <vq2.h>

typedef vq2::algo::gngt::Unit<Point>                    Unit;
typedef vq2::Graph<Unit,char,Unit::copy_constructor>    Graph;
typedef Graph::vertex_type                              Vertex;
typedef Graph::edge_type                                Edge;
typedef Graph::ref_vertex_type                          RefVertex;
typedef Graph::ref_edge_type                            RefEdge;

class AI
{
private:
	GR::DyeFilter *dyeFilter;
	image_transport::Publisher pubImageFiltrer;

public:
	AI();
	~AI();

	void imageCallBack(const sensor_msgs::ImageConstPtr& msg);

	struct Point // nested class
	{
		double x, y;
	};

	class Similarity {
	public:
	  typedef Point value_type;
	  typedef Point sample_type;

	  double operator()(const value_type& arg1,
	                    const sample_type& arg2) {
	    double dx = arg1.x - arg2.x;
	    double dy = arg1.y - arg2.y;
	    return dx*dx + dy*dy;
	  }
	};
	typedef vq2::unit::Similarity<Unit,Similarity> UnitSimilarity;

	class Learn {
	public:
	  typedef Point sample_type;
	  typedef Point weight_type;
	  void operator()(double coef,
	                  weight_type& prototype,
	                  const sample_type& target) {
	    prototype.x += coef * (target.x - prototype.x);
	    prototype.y += coef * (target.y - prototype.y);
	  }
	};
	typedef vq2::unit::Learn<Unit,Learn> UnitLearn;

		// Parametres de l'algo a bien comprendre....
	#define NB_SAMPLES 20000
	#define TARGET      2e-4
	class Params {
	public:

	  // GNG-T
	  int ageMax(void)           {return 20;}
	  double learningRate(void)  {return .001;}
	  double learningRatio(void) {return .2;}
	  double lambda(void)        {return .001;}
	  double infinity(void)      {return 1e12;}

	  // Evolution
	  double target(void)        {return TARGET;}
	  int nbSamples(void)        {return NB_SAMPLES;}
	  double lowPassCoef(void)   {return .4;}
	  double delta(void)         {return .75;}
	  double margin(void)        {return .2;}
	}; 
	typedef vq2::by_default::gngt::Evolution<Params> Evolution;

	const Point& sample_of(const Point& p) {return p;}



	class DisplayEdge {
		cv::Mat& rImage;
	public:
		DisplayEdge(cv::Mat &img) : rImage(img) {}

	  bool operator()(Edge& e) {
	    Point A = (*(e.n1)).value.prototype();
	    Point B = (*(e.n2)).value.prototype();

	    

	    return false; // the element should not be removed.
	  }
	};

	// This is a loop functor class.
	class DisplayVertex {
	  unsigned char* img; // reference sur l'image opencv
	public:
	  bool operator()(Vertex& n) { 
	    Point A = n.value.prototype();
	    // Bla bla
	    return false; // the element should not be removed.
	  }
	};

};