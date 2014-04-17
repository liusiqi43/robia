
#include <vector>
#include <utility>
#include <algorithm>


typedef std::pair<double, double> Point; // pt.first = x, pt.second = y

typedef vq2::algo::gngt::Unit<Point>                    Unit;
typedef vq2::Graph<Unit,char,Unit::copy_constructor>    Graph;
typedef Graph::vertex_type                              Vertex;
typedef Graph::edge_type                                Edge;
typedef Graph::ref_vertex_type                          RefVertex;
typedef Graph::ref_edge_type                            RefEdge;



class Similarity {
public:
  typedef Point value_type;
  typedef Point sample_type;

  double operator()(const value_type& arg1,
                    const sample_type& arg2) {
    double dx = arg1.first - arg2.first;
    double dy = arg1.second - arg2.second;
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
    prototype.first += coef * (target.first - prototype.first);
    prototype.second += coef * (target.second - prototype.second);
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

void fill(std::vector<Point>& points) {
  double x,y;

  points.clear();
  for(;;/* points de l'image */)
    if(true /* filtre du point */)
      p.push_back(std::make_pair(x,y));
  
  
}


class DisplayEdge {
  unsigned char* img; // reference sur l'image opencv
public:
  bool operator()(Edge& e) {
    Point A = (*(e.n1)).value.prototype();
    Point B = (*(e.n2)).value.prototype();

    // Bla bla

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



// This is a loop functor class.
class ComputeG {
private:
  Point G;
  unsigned int nb;
public:
  ComputeG(void) : G({0,0}), nb(0) {}

  bool operator()(Vertex& n) { 
    Point A = n.value.prototype();
    // Bla bla
    return false; // the element should not be removed.
  }
  
  Point barycenter(void) {
    return {G.x/nb,G.y/nb};
  }
  
};






#define NB_EPOCHS_PER_FRAME 10

int main(int argc, char* argv[]) {

  std::vector<Point> points;

  Params           params;
  Similarity       distance;
  UnitSimilarity   unit_distance(distance);
  Learn            learn;
  UnitLearn        unit_learn(learn);
  Evolution        evolution(params);

  while(true) {
    fill(points); // points de l'image


    for(int e = 0; e < NB_EPOCHS_PER_FRAME; +=e) {
      std::shuffle(points.begin(),points.end()); // Important !!!!
      auto begin = points.begin();
      auto end   = points.begin()+100; // si je veux 100 points au max.... Bug (si pas 100 points).
      vq2::algo::gngt::open_epoch(g,evolution);
      for(auto iter = begin; iter != end; +=iter)
	vq2::algo::gngt::submit(params,g,
				unit_distance,unit_learn,
				*iter,true);
      vq2::algo::gngt::close_epoch(params,g,
                                   unit_learn,
                                   evolution,true);
    }

    // dessin : il faut parcourrir le graphe....
    DisplayVertex display_v;
    display_v.img = 0; // la référence sur l'image
    DisplayEdge display_e;
    display_e.img = 0; // la référence sur l'image

    g.for_each_vertex(display_v);
    g.for_each_edge(display_e);


    // Composantes connexes
    std::map<unsigned int,Graph::Component*> components;
    g.computeConnectedComponents(components,true); // true... inutile, ça conserve des labels intelligemment. Mettre false.
    
    
    for(auto iter = components.begin();iter != components.end();++iter) {
      std::cout << "Label " << (*iter).first << std::endl
		<< std::endl;
      auto comp = (*iter).second;
      ComputeG compute_g;
      comp->for_each_vertex(compute_g);
      Point G = compute_g.barycenter();


      // comp->for_each_vertex(display_v);
      // comp->for_each_edge(display_e); // inutile...
    }
  }

  
}
