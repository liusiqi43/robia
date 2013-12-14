/*   This file is part of vq2
 *
 *   Copyright (C) 2012,  Supelec
 *
 *   Author : Herve Frezza-Buet
 *
 *   Contributor :
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public
 *   License (GPL) as published by the Free Software Foundation; either
 *   version 3 of the License, or any later version.
 *   
 *   This library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *   General Public License for more details.
 *   
 *   You should have received a copy of the GNU General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *   Contact : Herve.Frezza-Buet@supelec.fr
 *
 */


#include <vq2.h>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstdlib>
#include <vector>
#include <cmath>

// Let us use 2D points as prototypes.
// Our GNG-T will contain 2D points as prototypes.
class Point {
public:
  double x,y;
  
  Point(void) : x(0), y(0) {}
  Point(double xx, double yy) : x(xx), y(yy) {}
  Point(const Point& p) : x(p.x), y(p.y)       {}
  Point& operator=(const Point& p) {
    if(this != &p) {
      x = p.x;
      y = p.y;
    }
    return *this;
  }

  Point operator+(const Point& p) const {
    return Point(x + p.x, y + p.y);
  }

  // We will obtain vector operation from the default class. So our
  // type must implement those methods.

  Point& operator=(double value) {
    x = value;
    y = value;
    return *this;
  }

  Point& operator+=(const Point& p) {
    x += p.x;
    y += p.y;
    return *this;
  }

  Point& operator/=(double scale) {
    x /= scale;
    y /= scale;
    return *this;
  }

  Point& operator*=(double scale) {
    x *= scale;
    y *= scale;
    return *this;
  }
};

// This is the GNGT neurons.
typedef vq2::algo::gngt::Unit<Point> Unit;
// Do not forget the vertex initialization functor (3rd argument).
typedef vq2::Graph<Unit,char,Unit::copy_constructor>    Graph;


// For finding the winner in the GNGT algorithm, we need some
// similarity measure. Here, let us use the squared euclidian
// distance.
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
typedef vq2::unit::UnitToSampleSimilarity<Unit,Similarity> UnitSimilarity;

// The learning process.
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


// This converts a Point into a 2D position on some A4 sheet paper.
class FigConverter {
public:
  typedef Point position_type;
  
  void locate(const position_type& pos,
	      double& x_cm, double& y_cm) {
    x_cm = 21.0/2 + 2*pos.x;
    y_cm = 29.7/2 + 2*pos.y;
  }
};
typedef vq2::unit::XfigConverter<Unit,FigConverter> UnitFigConverter;

// This is the parameter set for GNG-T
class Params {
public:
  int ageMax(void)                {return 20;}
  double firstLearningRate(void)  {return .01;}
  double secondLearningRate(void) {return .2*firstLearningRate();}
  double lambda(void)             {return .001;}
  double infinity(void)           {return 1e12;}
}; 

// This class is required to control the GNG-T evolution. It fits
// vq2::concept::GNGTEvolution.

#define NB_SAMPLES 20000
#define TARGET  4
#define GNUPLOT_TARGET_SCALE 5
#define CONFIDENCE .75
#define EVOLUTION_DECISION_THRES 3

// A value extraction functor.... identity here.
double value_of(double x) {return x;}

class Evolution {
public:

  std::vector<double> disto_distrib;
  
  // Bounds of the confidence interval.
  double min,max;
  // Decisions for nb of vertices evolution.
  unsigned int add,remove;

  Evolution(void) 
  : add(0), remove(0) {}

  void clear(void)              {disto_distrib.clear();}
  void operator+=(double value) {disto_distrib.push_back(value);}


  // returns -1 if there are too many prototypes, 1 if prototypes should be added, 0 otherwise.
  int operator()() {
    int res = 0;
    // Let us find where main values of the distortion distribution stands.
    vq2::proba::shortest_confidence_interval(disto_distrib.begin(),
					     disto_distrib.end(),
					     value_of,
					     CONFIDENCE,min,max);
    if(TARGET<min) {
      remove = 0;
      ++add;
      if(add >= EVOLUTION_DECISION_THRES)
	res = 1;
    }
    else if(TARGET>max) {
      ++remove;
      add = 0;
      if(remove >= EVOLUTION_DECISION_THRES)
	res = -1;
    }
    else {
      remove = 0;
      add = 0;
    }
    return res;
  }
};

// Let us gather density manipulations in some class.
class Input {
private:

  typedef vq2::proba::Density<Point> Density;

  class Box : public Density {
  public:
    typedef Point event_type;
    Point min,max;
    double density;
    bool enabled;
  
    Box(const Point& origin,
	double width, double height,
	double proba_density)
      : min(origin), max(origin+Point(width,height)),
	density(proba_density),
	enabled(false) {}

    virtual double operator()(const event_type& evt) {
      if(enabled 
	 && evt.x >= min.x
	 && evt.x <  max.x
	 && evt.y >= min.y
	 && evt.y <  max.y)
	return density;
      return 0;
    }
  };


 
  Box&     box1; 
  Box&     box2; 
  Box&     noise;
  Density& glue1;
  Density& glue2;

public:

  std::vector<Point> samples;

  Input(void) 
    : box1(*(new Box(Point(1,1),1,1,1))),
      box2(*(new Box(Point(3,1),1,1,.5))),
      noise(*(new Box(Point(0,0),5,3,.05))),
      glue1(vq2::proba::max((Density&)box1,(Density&)box2)),
      glue2(vq2::proba::max(glue1,(Density&)noise)) {
  }
  
  ~Input(void) {
    delete &box1; 
    delete &box2; 
    delete &noise;
    delete &glue1;
    delete &glue2;
  }

#define INPUT_STAGE_LENGTH     20
  void acquireSamples(int t) {
    Point p;
    
    int stage = t / INPUT_STAGE_LENGTH;
    int phase = stage % 4;

    switch(phase) {
    case 0:
      box1.enabled  = true;
      box2.enabled  = false;
      noise.enabled = false;
      break;
    case 1:
      box1.enabled  = true;
      box2.enabled  = true;
      noise.enabled = false;
      break;
    case 2:
      box1.enabled  = true;
      box2.enabled  = true;
      noise.enabled = true;
      break;
    case 3:
      box1.enabled  = true;
      box2.enabled  = false;
      noise.enabled = true;
      break;
    }

    samples.clear();
    for(int i=0; i<NB_SAMPLES; ++i) {
      p.x = vq2::proba::random::uniform(0,5);
      p.y = vq2::proba::random::uniform(0,3);
      if(vq2::proba::random::toss(glue2(p)))
	samples.push_back(p);
    }
  }
};

// These are some functions used inside the main, and defined
// afterwards.
std::string toString(int nb);
void make_epoch(int               t,
		Params&           p, 
		Graph&            g, 
		UnitSimilarity&   distance,
		UnitLearn&        learn,
		Evolution&        evolution,
		Input&            input,
		std::ofstream&    file); 
void print(int     nb,
	   Graph&  g, 
	   Input&  input); 
void generateMakefile(void);
void generateGnuplot(void);

#define NB_TICKS             160
#define NB_EPOCHS_PER_TICK    10
int main(int argc, char* argv[]) {
  Graph g;
  Params           params;
  Similarity       distance;
  UnitSimilarity   unit_distance(distance);
  Learn            learn;
  UnitLearn        unit_learn(learn);
  Evolution        evolution;
  Input            input;
  int              epoch,tick;
  std::ofstream    gnuplot;

  generateMakefile();

  vq2::proba::random::init();

  gnuplot.open("gngt-target.data");
  if(!gnuplot) {
    std::cout << "Cannot open \"gngt-target.data. Aborting" << std::endl;
    exit(0);
  }

  generateGnuplot();

  for(tick = 0; tick < NB_TICKS; ++tick) {
    for(epoch = 0; epoch < NB_EPOCHS_PER_TICK; ++epoch) 
      make_epoch(tick,params,g,unit_distance,unit_learn,evolution,input,gnuplot);
    print(tick,g,input);
  }

  gnuplot.close();
  std::cout << std::endl
	    << std::endl
	    << "\"gngt-target.plot\" generated." << std::endl
	    << std::endl
	    << "type 'make' to make the movie." << std::endl;
}


std::string toString(int nb) {
  std::ostringstream ostr;
  ostr << std::setw(6) << std::setfill('0') << nb;
  return ostr.str();
}

// This class validates all neurons and edges. This is a for_each_edge functor.
class ValidateEdge {
public:
  bool operator()(Graph::edge_type& e) { 
    e.stuff.efficient = true;
    return false; 
  }
};

// This class validates all neurons and edges. This is a for_each_vertex functor.
class ValidateVertex {
public:
  bool operator()(Graph::vertex_type& v) { 
    ValidateEdge validate_edge;
    v.stuff.efficient = true;
    v.for_each_edge(validate_edge);
    return false; 
  }
};

// This class invalidates long edges. This is a for_each_edge functor.
#define MAX_VALID_DISTANCE .5
#define MAX_VALID_SQUARED_DISTANCE (MAX_VALID_DISTANCE*MAX_VALID_DISTANCE)
class InvalidateLongEdge {
public:
  bool operator()(Graph::edge_type& e) { 
    Similarity squared_dist;
    Point& A = (*(e.n1)).value.prototype(); // This is the way to retrieve the weight
    Point& B = (*(e.n2)).value.prototype();
    e.stuff.efficient = squared_dist(A,B) < MAX_VALID_SQUARED_DISTANCE;
    return false; // We invalidate, but we keep the edge.
  }
};


// This tells whether some valid edges exist.
class IsThereValidEdges {
public:
  bool is_any_valid;

  IsThereValidEdges(void) : is_any_valid(false) {}
  bool operator()(Graph::edge_type& e) { 
    is_any_valid = is_any_valid || e.stuff.efficient;
    return false;
  }
};

// This invalidates a vertex if all its edges are invalid.
class InvalidateNoisyVertex {
public:
  bool is_any_valid;

  bool operator()(Graph::vertex_type& v) { 
    IsThereValidEdges search;
    v.for_each_edge(search);
    v.stuff.efficient = search.is_any_valid;
    return false;
  }
};


void make_epoch(int               t,
		Params&           p, 
		Graph&            g, 
		UnitSimilarity&   distance,
		UnitLearn&        learn,
		Evolution&        evolution,		
		Input&            input,
		std::ofstream&    gnuplot) {
  std::vector<Point>::iterator iter,end;
  static int e=0;

  
  // Let us acquire the input samples for that epoch.
  input.acquireSamples(t);
  vq2::algo::gngt::open_epoch(g,evolution);
  for(iter=input.samples.begin(),end=input.samples.end();
      iter != end;
      ++iter)
    vq2::algo::gngt::submit(p,g,distance,learn,*iter,true);
  vq2::algo::gngt::close_epoch(p,g,learn,evolution,true);
  
  gnuplot << std::setw(3) << e++ 
	  << ' ' << GNUPLOT_TARGET_SCALE*TARGET
	  << ' ' << std::setw(3) << evolution.disto_distrib.size() 
	  << ' ' << GNUPLOT_TARGET_SCALE*evolution.min
	  << ' ' << GNUPLOT_TARGET_SCALE*evolution.max << std::endl;
}

#define VERTEX_RADIUS     .2  // cm
#define VERTEX_THICKNESS   3  // 1/80 inches (see xfig thickness)
#define VERTEX_DEPTH      50  // xfig depth
#define EDGE_THICKNESS     5  // 1/80 inches (see xfig thickness)
#define EDGE_DEPTH        51  // xfig depth
#define DATA_THICKNESS     2  // 1/80 inches (see xfig thickness)
#define DATA_DEPTH        52  // xfig depth
#define FRAME_DEPTH      999  // xfig depth
void print(int     nb,
	   Graph&  g, 
	   Input&  input) {
  FigConverter converter;
  UnitFigConverter unit_converter(converter);
  std::ofstream file;
  vq2::xfig::GC gc;

  std::cout << "Frame " << std::setw(4) << nb << "    \r" << std::flush;
  vq2::xfig::open("gngt",nb,file);

  // Let us draw an invisible frame surrounding the drawing.
  gc.depth = FRAME_DEPTH;
  gc.colorWhite();
  vq2::xfig::line(converter,file,gc,Point(-.5,-.5),Point(5.5,-.5));
  vq2::xfig::line(converter,file,gc,Point(5.5,-.5),Point(5.5,3.5));
  vq2::xfig::line(converter,file,gc,Point(5.5,3.5),Point(-.5,3.5));
  vq2::xfig::line(converter,file,gc,Point(-.5,3.5),Point(-.5,-.5));

  // Let us draw inputs;
  gc.depth = DATA_DEPTH;
  gc.thickness = DATA_THICKNESS;
  gc.colorBlack();
  std::vector<Point>::iterator iter,end;
  for(iter=input.samples.begin(),end=input.samples.end();
      iter != end;
      ++iter)
    vq2::xfig::point(converter,file,gc,*iter);

  // Let us draw the graph
  vq2::xfig::graph(unit_converter,file,g,
		   VERTEX_RADIUS,VERTEX_THICKNESS,VERTEX_DEPTH,
		   EDGE_THICKNESS,EDGE_DEPTH);
  
  vq2::xfig::close(file);
}

void generateMakefile(void) {
  std::ofstream file;
  
  file.open("Makefile");

  file << "FIG = $(wildcard gngt-*.fig)" << std::endl
       << "JPG = $(sort $(patsubst %.fig, %.jpg, $(FIG)))" << std::endl
       << std::endl
       << "help:" << std::endl
       << "\t@echo \"fig2dev and ffmpeg are needed.\"" << std::endl
       << "\t@echo" << std::endl
       << "\t@echo \"type one of the following commands\"" << std::endl
       << "\t@echo \"    make jpeg\"" << std::endl
       << "\t@echo \"    make clear\"" << std::endl
       << "\t@echo \"    make plot\"" << std::endl
       << "\t@echo \"    make movie\"" << std::endl
       << "\t@echo" << std::endl
       << std::endl
       << "clear:" << std::endl
       << "\t@rm gngt-*.jpg gngt-*.fig" << std::endl
       << std::endl
       << "gngt-%.jpg : gngt-%.fig" << std::endl
       << "\t@fig2dev -L jpeg -q 100 $< $@" << std::endl
       << "\t@echo $@ \"generated.\"" << std::endl
       << std::endl
       << "jpeg : $(JPG)" << std::endl
       << std::endl
       << "plot :" << std::endl
       << "\t@gnuplot -p gngt-target.plot" << std::endl
       << std::endl
       << "movie : jpeg" << std::endl
       << "\t@rm -f gngt.avi" << std::endl
       << "\tffmpeg -r 8 -i gngt-%06d.jpg -b 1M gngt.avi" << std::endl
       << "\t@rm -f $(JPG) $(FIG) Makefile" << std::endl;


  file.close();

}

void generateGnuplot(void) {
  std::ofstream gnuplot;
  
  gnuplot.open("gngt-target.plot");
  if(!gnuplot) {
    std::cout << "Cannot open \"gngt-target.plot. Aborting" << std::endl;
    exit(0);
  }
  gnuplot << "set term wxt dashed" << std::endl
	  << "set title \"GNG-T stats\"" << std::endl
	  << "set yrange [0:50]" << std::endl
	  << "plot 'gngt-target.data' using 1:2 with lines lt 1 lc 1 title '" << GNUPLOT_TARGET_SCALE << "*target', 'gngt-target.data' using 1:3 with lines lt 1 lc 2 title 'nb nodes', 'gngt-target.data' using 1:4 with lines lt 3 lc 7 title 'confidence', 'gngt-target.data' using 1:5 with lines lt 3 lc 7 notitle" << std::endl;

  gnuplot.close();
}
