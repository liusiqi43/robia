#pragma once


/**
 * @example example-001-conversion.cc
 */
/**
 * @example example-002-colortest.cc
 */


#include <cmath>
#include <mirage.h>
#include <iostream>
#include <algorithm>
#include <map>
#include <vector>
#include <string>
#include <fstream>


namespace dye {

  
  /**
   * This returns the color condiguous index in [0,6] of the rgb triplet. In this triplet, one of the components have to be null.
   */
  template<typename COMPONENT> 
  double indexOf(COMPONENT r, COMPONENT g, COMPONENT b) {
    int area = 0;
    double res;

    if(r>=g)
      area += 4;
    if(r>=b)
      area += 2;
    if(g>=b)
      area += 1;

    switch(area) {
    case 0: // (r,g,b) ~ (0,x,1)
      res =         g/(double)b;
      break;
    case 1: // (r,g,b) ~ (0,1,x)
      res = 1 + (g-b)/(double)g;
      break;
    case 3: // (r,g,b) ~ (x,1,0)
      res = 2 +     r/(double)g;
      break;
    case 7: // (r,g,b) ~ (1,x,0)
      res = 3 + (r-g)/(double)r;
      break;
    case 6: // (r,g,b) ~ (1,0,x)
      res = 4 +     b/(double)r;
      break;
    case 4: // (r,g,b) ~ (x,0,1)
      res = 5 + (b-r)/(double)b;
      break;
    default:
      res = 0;
      std::cerr << "Error : dye::colorOf : bad rgb triplet as input." << std::endl;
    }

    return res;
  }

  enum {
    darkIndex = -1
  };

  /**
   * Computes a rgb from a value. The input value is either
   * dark_value, either in [0,6].
   */
  inline void rgbOf(double value,mirage::colorspace::RGB_24& rgb) {
    if(value == darkIndex) {
      rgb = mirage::colorspace::RGB_24(0,0,0);
      return;
    }

    if(value < 1.0) {
      rgb = mirage::colorspace::RGB_24(0,
				       (unsigned char)(255*value+.5),
				       255);
      return;
    }
   
    if(value < 2.0) {
      rgb = mirage::colorspace::RGB_24(0,
				       255,
				       (unsigned char)(255*(2.0-value)+.5));
      return;
    }
   
    if(value < 3.0) {
      rgb = mirage::colorspace::RGB_24((unsigned char)(255*(value-2.0)+.5),
				       255,
				       0);
      return;
    }
   
    if(value < 4.0) {
      rgb = mirage::colorspace::RGB_24(255,
				       (unsigned char)(255*(4.0-value)+.5),
				       0);
      return;
    }
   
    if(value < 5.0) {
      rgb = mirage::colorspace::RGB_24(255,
				       0,
				       (unsigned char)(255*(value-4.0)+.5));
      return;
    }

    rgb = mirage::colorspace::RGB_24((unsigned char)(255*(6-value)+.5),
				     0,
				     255);
  }


  /**
   * Keeps only color information in some rgb.
   * @return true if the rgb output value is dark.
   */
  template<typename RGBin,typename RGBout>
  bool removeColor(const RGBin& in,
		   RGBout& out,
		   typename RGBin::value_type dark_threshold) {
    typename RGBin::value_type inf = in._red;
    if(in._green < inf) inf = in._green;
    if(in._blue < inf) inf = in._blue;
    if((out._red = in._red-inf)<dark_threshold) out._red = 0;
    if((out._green = in._green-inf)<dark_threshold) out._green = 0;
    if((out._blue = in._blue-inf)<dark_threshold) out._blue = 0;

    return out._red == 0 && out._green == 0 && out._blue == 0;
  }
   
  

  /**
   * This computes the indexes of an image. The input image is some
   * rgb image, and the output is a scalar image, which values are an
   * index (in [0,6]) or -1 in case of a dark pixel (without coulor).
   * @param input The input image (rgb).
   * @param output The output image (scalar)
   * @param dark_threshold A rgb component under this threshold is considered as null.
   * @param dark_value The output image pixel value for dark areas.
   */
  template<typename INPUT_IMAGE,
	   typename OUTPUT_IMAGE>
  void index(const INPUT_IMAGE& input,
	     OUTPUT_IMAGE& output,
	     typename INPUT_IMAGE::value_type::value_type dark_threshold) {
    typename INPUT_IMAGE::const_pixel_type it,end;
    typename OUTPUT_IMAGE::pixel_type ot;
    
    output.resize(input._dimension);
    for(it = input.const_begin(), end = input.const_end(), ot = output.begin();
	it != end;
	++it, ++ot) {
      const typename INPUT_IMAGE::value_type& val = *it; 
      typename INPUT_IMAGE::value_type rgb;

      if(removeColor(val,rgb,dark_threshold))
	*ot = darkIndex;
      else
	*ot = indexOf(rgb._red,rgb._green,rgb._blue);
    }
  }

  /**
   * This sets an histogram of the color values.
   * h[bin_center] = %allpixels.
   */
  template<typename INPUT_IMAGE>
  void indexHistogram(const INPUT_IMAGE& input,
		      std::map<double,double>& histogram,
		      typename INPUT_IMAGE::value_type::value_type dark_threshold,
		      int nb_bins) {
    double step = 6.0/nb_bins;
    histogram.clear();
    double size = input._dimension[0]*input._dimension[1];
    std::vector<unsigned int> h(nb_bins);
    std::vector<unsigned int>::iterator b,end;

    for(b=h.begin(),end=h.end();b!=end;++b)
      *b = 0;

    typename INPUT_IMAGE::const_pixel_type pix,pixend;
    for(pix=input.const_begin(),pixend=input.const_end();pix!=pixend;++pix) {
      const typename INPUT_IMAGE::value_type& val = *pix; 
      typename INPUT_IMAGE::value_type rgb;

      if(!removeColor(val,rgb,dark_threshold)) {
	int bin = (int)(nb_bins*indexOf(rgb._red,rgb._green,rgb._blue)/6.0);
	if(bin==nb_bins)
	  --bin;
	++(h[bin]);
      }
    }

    for(int i=0;i<nb_bins;++i)
      histogram[(i+.5)*step]=h[i]/size;
  }

  /**
   * This gnuplots an index histogram.
   */
  inline void plot(const std::map<double,double>& histogram,
		   std::string filename) {
    std::ofstream file;
    file.open(filename.c_str());
    if(!file) {
      std::cerr << "Cannot open \"" << filename << "\". Aborting." << std::endl;
      return;
    }

    double ymax=0;
    double y;
    std::map<double,double>::const_iterator iter,end;
    for(iter = histogram.begin(), end = histogram.end();
	iter != end;
	++iter)
      if((y=100*(*iter).second)>ymax)
	ymax=y;

    file << "set title  \"Color index histogram\"" << std::endl
	 << "set xlabel \"Color index\"" << std::endl
	 << "set ylabel \"% of total pixels\"" << std::endl
	 << "set cbrange [0:6]" << std::endl
	 << "set xrange [0:6]" << std::endl
	 << "set yrange [0:" << ymax << "]" << std::endl
	 << "set view map" << std::endl
	 << "set style function pm3d" << std::endl
	 << "set palette defined ( ";
    
    mirage::colorspace::RGB_24 rgb;
    double val;
    double first=true;
    for(val=0;val<=6;val+=.1,first=false) {
      if(!first)
	file << ", ";
      rgbOf(val,rgb);
      file << val 
	   << ' ' << rgb._red/255.0 
	   << ' ' << rgb._green/255.0 
	   << ' ' << rgb._blue/255.0;
    }

    file << " )" << std::endl
	 << "unset colorbox" << std::endl
	 << "splot x with pm3d, '-' using 1:2:3 with linespoints notitle pt 7 ps 0 lw 3 lc rgb '#000000'" << std::endl;


    for(iter = histogram.begin(), end = histogram.end();
	iter != end;
	++iter)
      file << (*iter).first << ' ' << 100*(*iter).second << " 7" << std::endl;
    file.close();
  }

  /**
   * This tells wether two indexes can be considered as the same color.
   */
  inline bool index_fit(double i1, double i2, double tolerance) {
    if(i1 == darkIndex || i2 == darkIndex)
      return i1 == i2;

    double d = fabs(i1-i2);
    return  std::min(d,6-d) < tolerance;
  }

  /**
   * This tells wether a pixel has some given color.
   * PARAM::colorIndex() should return the color index of the
   * reference color (in [0,6]). Let us note i the index of the rgb pixel tested, the test returns true when |i-PARAM::colorIndex()| < PARAM::tolerance(). Last, PARAM::darkThreshold() returns the darkness threshold. Its type is the type of some rgb components.
   */
  template<typename PARAM,typename RGB>
  class Test {
  public:
    bool operator()(const RGB& val) const {
      RGB rgb;
      if(removeColor(val,rgb,PARAM::darkThreshold()))
	return false;
      return index_fit(PARAM::colorIndex(),indexOf(rgb._red,rgb._green,rgb._blue),PARAM::tolerance());
    }
  };

  
  template<typename INPUT_IMAGE,
	   typename OUTPUT_IMAGE,
	   typename TEST>
  void mask(const INPUT_IMAGE& input,
	    OUTPUT_IMAGE& output,
	    const TEST& test,
	    typename OUTPUT_IMAGE::value_type true_value,
	    typename OUTPUT_IMAGE::value_type false_value) {
    typename INPUT_IMAGE::const_pixel_type i,end;
    typename OUTPUT_IMAGE::pixel_type o;
    output.resize(input._dimension);

    for(i=input.const_begin(),o=output.begin(),end=input.const_end();
	i != end;
	++i, ++o)
      if(test(*i))
	*o = true_value;
      else
	*o = false_value;
  }

  
	    
}
