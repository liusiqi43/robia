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

#ifndef vq2GNGT_H
#define vq2GNGT_H

#include <vq2Concept.h>
#include <vq2Closest.h>
#include <vq2Functors.h>
#include <vq2Graph.h>
#include <iostream>
#include <cmath>
#include <string>

namespace vq2 {
  namespace algo {
    namespace gngt {

      template<typename PROTOTYPE>
      class Unit : public unit::Base<PROTOTYPE>{
      public:
	
	/**
	 * The distorion accumulator.
	 */
	double e;
	
	/**
	 * The number of times this prototype was the best during last
	 * epoch.
	 */
	int n;
	
	typedef typename unit::Base<PROTOTYPE>::prototype_type prototype_type;

	Unit(void) 
	  : unit::Base<PROTOTYPE>(), 
	    e(0), n(0) {}
	Unit(const Unit<PROTOTYPE>& copy) 
	  : unit::Base<PROTOTYPE>(copy), 
	    e(copy.e), n(copy.n) {}
	Unit(const prototype_type& proto) : 
	  unit::Base<PROTOTYPE>(proto), 
	  e(0), n(0) {}

	Unit<PROTOTYPE>& operator=(const Unit& copy) {
	  if(this != &copy) {
	    this->unit::Base<PROTOTYPE>::operator=(copy);
	    e = copy.e;
	    n = copy.n;
	  }
	  return *this;
	}

	Unit<PROTOTYPE>& operator=(const prototype_type& proto) {
	  
	  this->unit::Base<PROTOTYPE>::operator=(proto);
	  e = 0;
	  n = 0;
	  return *this;
	}

	class copy_constructor {
	public:
	  void operator()(Unit<PROTOTYPE>& that,
			  const Unit<PROTOTYPE>& copy) {
	    that = copy;
	  }
	};
      };

      namespace internal {

	template<typename VERTEX>
	class ClearNeuronStats {
	private:
	public:
	  ClearNeuronStats(void) {}
	  bool operator()(VERTEX& v) {
	    v.value.n = 0;
	    v.value.e = 0;
	    return false; 
	  }
	};
	
	template<typename VERTEX,typename REF_VERTEX,
		 typename EDGE,
		 typename GNGT_PARAMS>
	class UpdateAge {
	public:
	  bool found;
	  VERTEX* n2;
	  GNGT_PARAMS& params;
	  
	  UpdateAge(REF_VERTEX second,
		    GNGT_PARAMS& p) 
	    : found(false), 
	      n2(&(*second)), 
	      params(p) {}
	  
	  bool operator()(EDGE& e) {
	    if((&(*(e.n1)) == n2) || (&(*(e.n2)) == n2)) {
	      e.value = 0;
	      found = true;
	    }
	    else {
	      e.value++;
	      if(e.value > params.ageMax())
		return true;
	    }
	    return false;
	  }
	};

	template<typename VERTEX,typename REF_VERTEX,
		 typename EDGE,
		 typename LEARN,
		 typename GNGT_PARAMS>
	class UpdateNeighbour {
	public:
	  VERTEX* n1;
	  GNGT_PARAMS& params;
	  LEARN& learn;
	  const typename LEARN::sample_type& xi;
	  
	  UpdateNeighbour(REF_VERTEX& first,
			  const typename LEARN::sample_type& x,
			  GNGT_PARAMS& p,
			  LEARN& l) 
	    : n1(&(*first)), 
	      params(p),
	      learn(l),
	      xi(x) {}
	  
	  bool operator()(EDGE& e) {
	    VERTEX* n2;
	    if(&(*(e.n1)) == n1)
	      n2 = &(*(e.n2));
	    else
	      n2 = &(*(e.n1));
	    
	    learn(params.secondLearningRate(),n2->value,xi);
	    return false;
	  }
	};


	template<typename REF_VERTEX,
		 typename UNIT,
		 typename GNGT_EVOLUTION>
	class MakeNeuronStat {
	public:

	  GNGT_EVOLUTION& stats;
	  REF_VERTEX min,max;
	  double mine,maxe;
	  unsigned int& nb;

	  MakeNeuronStat(GNGT_EVOLUTION& e,
			 unsigned int& nb_vertices) 
	    : stats(e), 
	      min(), max(), 
	      mine(-1), maxe(0),
	      nb(nb_vertices) {
	    nb = 0;
	    stats.clear();
	  }
	  bool operator()(REF_VERTEX& ref) {
	    UNIT& proto = (*ref).value;
	    if(proto.n == 0)
	      return true;
	    
	    nb++;
	    stats += proto.e;


	    if(proto.e < mine || mine < 0) {
	      mine = proto.e;
	      min = ref;
	    }

	    // Not elseif... is a single neuron is there
	    // >= in case of e=0...
	    if(proto.e >= maxe) {
	      maxe = proto.e;
	      max = ref;
	    }
	    return false;
	  }
	};

	template<typename VERTEX,typename REF_VERTEX,
		 typename EDGE,
		 typename GNGT_PARAMS>
	class FindHighestNeighbour {
	public:

	  REF_VERTEX& n1;
	  REF_VERTEX  n2;
	  double max;
	  

	  FindHighestNeighbour(REF_VERTEX& max) 
	    : n1(max), n2(), max(0) {}

	  bool operator()(EDGE& e) {
	    REF_VERTEX n;
	    if(&(*(e.n1)) == &(*n1))
	      n = e.n2;
	    else
	      n = e.n1;
	    
	    if((*n).value.e >= max) {
	      max = (*n).value.e;
	      n2 = n;
	    }
	    return false;
	  }
	};
      }
      
	
      template<typename GRAPH,
	       typename GNGT_EVOLUTION>
      void open_epoch(GRAPH& g,
		      GNGT_EVOLUTION& stats) {
	internal::ClearNeuronStats<typename GRAPH::vertex_type> clear_vertex_stats;
	g.for_each_vertex(clear_vertex_stats);
      }
      
      /**
       * Once StartEpoch is called, use this method to submit examples.
       * @param dynamic_topology true means that topology (number of vertices and edges) can change.
       */
      template<typename GRAPH,typename GNGT_PARAMS,
	       typename SIMILARITY, typename LEARN>
      static void submit(GNGT_PARAMS& params,
			 GRAPH& g,
			 SIMILARITY& distance,
			 LEARN& learn,
			 const typename LEARN::sample_type& xi,
			 bool dynamic_topology) {

	// Create two first vertextes from xi.
	if(g.nbVertices() < 2) {
	  if(dynamic_topology) 
	    typename GRAPH::ref_vertex_type ref(g += typename GRAPH::vertex_value_type(xi));
	  return;
	}

	// Find the two closest vertices.
	std::pair<double,double> closest_dists;
	std::pair<typename GRAPH::ref_vertex_type,
		  typename GRAPH::ref_vertex_type> two_closest = vq2::algo::twoClosest(g,distance,
										       params.infinity(),
										       xi,closest_dists);

	if(dynamic_topology) {
	  // Update edges.
	  internal::UpdateAge<typename GRAPH::vertex_type,
	    typename GRAPH::ref_vertex_type,
	    typename GRAPH::edge_type,
	    GNGT_PARAMS> update_age(two_closest.second,params);
	  (*(two_closest.first)).for_each_edge(update_age);
	  if(!(update_age.found))
	    g.connect(0,two_closest.first,two_closest.second);
	}

	// Update closest.
	typename GRAPH::vertex_value_type& n1 = (*(two_closest.first)).value;
	n1.e += closest_dists.first;
	++n1.n;
	learn(params.firstLearningRate(),n1,xi);

	// Update its neighbours.
	internal::UpdateNeighbour<typename GRAPH::vertex_type,
				  typename GRAPH::ref_vertex_type,
				  typename GRAPH::edge_type,
				  LEARN,
				  GNGT_PARAMS> update_neighbour(two_closest.first,
								xi,params,learn);
	(*(two_closest.first)).for_each_edge(update_neighbour);
      }

	
      /**
       * This close an epoch.
       * @param dynamic_topology true means that topology (number of vertices and edges) can change.
       */
      template<typename GNGT_PARAMS,
	       typename GRAPH,
	       typename LEARN,
	       typename GNGT_EVOLUTION>
      void close_epoch(GNGT_PARAMS& params,
		       GRAPH& g,
		       LEARN& learn,
		       GNGT_EVOLUTION& stats,
		       bool dynamic_topology) {
	unsigned int nb_vertices;
	internal::MakeNeuronStat<typename GRAPH::ref_vertex_type,
				 typename GRAPH::vertex_value_type,
				 GNGT_EVOLUTION> make_neuron_stat(stats,nb_vertices);
	  
	  
	if(!dynamic_topology)
	  return;

	g.for_each_vertex_ref(make_neuron_stat);
	if(nb_vertices == 0)
	  return;

	int increment = stats();
	if(increment == 0)
	  return;
	  
	// T is too low, we have to remove some neuron.
	if(increment < 0) {
	  make_neuron_stat.min.take();
	  make_neuron_stat.min.free();
	  return;
	}
	  
	// T is too high, we have to add some neuron.
	  
	// Let us find the neighbour with the highest e
	internal::FindHighestNeighbour<typename GRAPH::vertex_type,
				       typename GRAPH::ref_vertex_type,
				       typename GRAPH::edge_type,
				       GNGT_PARAMS>  find_highest_neighbour(make_neuron_stat.max);
	(*(make_neuron_stat.max)).for_each_edge(find_highest_neighbour);

	if(!(find_highest_neighbour.n2)) {
	  // max is an isolated node. We clone it.
	  typename GRAPH::ref_vertex_type clone(g += (*(make_neuron_stat.max)).value);
	  g.connect(0,make_neuron_stat.max,clone);
	}
	else {
	  typename GRAPH::vertex_value_type  c;
	  typename GRAPH::vertex_value_type& a = (*(find_highest_neighbour.n1)).value;
	  typename GRAPH::vertex_value_type& b = (*(find_highest_neighbour.n2)).value;
	  c.prototype() = a.prototype();
	  learn(params.lambda(),c,b.prototype());
	  typename GRAPH::ref_vertex_type middle(g += c);
	  g.connect(0,find_highest_neighbour.n1,middle);
	  g.connect(0,find_highest_neighbour.n2,middle);
	}
      }
    }
  }
}

#endif
