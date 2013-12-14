#pragma once

#include <cmath>
#include <iostream>
#include <algorithm>
#include <map>
#include <vector>
#include <string>
#include <fstream>
#include <opencv/cv.h>
#include <opencv/highgui.h>

/**
 * a modified version based on OpenCV BGR representation
 */

#define DARK_RGB_COMPONENT 50

namespace dye {
    typedef cv::Vec3b cvBGR;

    /**
     * This returns the color condiguous index in [0,6] of the bgr triplet. In this triplet, one of the components have to be null.
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
                    std::cerr << "Error : dye::colorOf : bad bgr triplet as input." << std::endl;
            }

            return res;
        }

    enum {
        darkIndex = -1
    };

    /**
     * Computes a bgr from a value. The input value is either
     * dark_value, either in [0,6].
     */
    inline void bgrOf(double value,cvBGR& bgr) {
        if(value == darkIndex) {
            bgr = cvBGR(0,0,0);
            return;
        }

        if(value < 1.0) {
            bgr = cvBGR(255,
                    (unsigned char)(255*value+.5),
                    0);
            return;
        }

        if(value < 2.0) {
            bgr = cvBGR((unsigned char)(255*(2.0-value)+.5),
                    255,
                    0);
            return;
        }

        if(value < 3.0) {
            bgr = cvBGR(0,
                    255,
                    (unsigned char)(255*(value-2.0)+.5));
            return;
        }

        if(value < 4.0) {
            bgr = cvBGR(0,
                    (unsigned char)(255*(4.0-value)+.5),
                    255);
            return;
        }

        if(value < 5.0) {
            bgr = cvBGR((unsigned char)(255*(value-4.0)+.5),
                    0,
                    255);
            return;
        }

        bgr = cvBGR(255,
                0,
                (unsigned char)(255*(6-value)+.5));
    }


    /**
     * Keeps only color information in some bgr.
     * @return true if the bgr output value is dark.
     */
        inline bool removeColor(const cvBGR& in,
                cvBGR& out,
                int dark_threshold) {
            int inf = in[2];
            if(in[1]< inf) inf = in[1];
            if(in[0] < inf) inf = in[0];
            if((out[2] = in[2]-inf)<dark_threshold) out[2] = 0;
            if((out[1] = in[1]-inf)<dark_threshold) out[1] = 0;
            if((out[0] = in[0]-inf)<dark_threshold) out[0] = 0;

            return out[2] == 0 && out[1] == 0 && out[0] == 0;
        }



    /**
     * This computes the indexes of an image. The input image is some
     * bgr image, and the output is a scalar image, which values are an
     * index (in [0,6]) or -1 in case of a dark pixel (without coulor).
     * @param input The input image (bgr).
     * @param output The output image (scalar)
     * @param dark_threshold A bgr component under this threshold is considered as null.
     * @param dark_value The output image pixel value for dark areas.
     */
            inline void index(const cv::Mat& input,
                    cv::Mat& output,
                    int dark_threshold) {

                cv::resize(output, output, input.size());
                cv::Vec3b vec;
                for(int i = 0; i < input.rows; i++)
                {
                    for(int j = 0; j < input.cols; j++)
                    {
                        cvBGR bgr = input.at<cvBGR>(i,j);
                        output.at<int>(i,j) = removeColor(bgr, vec, dark_threshold) ? 
                                                               darkIndex : indexOf(bgr[2], bgr[1], bgr[0]);
                    }
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
                typename INPUT_IMAGE::value_type bgr;

                if(!removeColor(val,bgr,dark_threshold)) {
                    // OpenCV store color in BGR order. therefore We need to sotre/read it in BGR order for processing
                    int bin = (int)(nb_bins*indexOf(bgr[2],bgr[1],bgr[0])/6.0);
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

        cvBGR bgr;
        double val;
        double first=true;
        for(val=0;val<=6;val+=.1,first=false) {
            if(!first)
                file << ", ";
            bgrOf(val,bgr);
            file << val 
                << ' ' << bgr[0]/255.0 
                << ' ' << bgr[1]/255.0 
                << ' ' << bgr[2]/255.0;
        }

        file << " )" << std::endl
            << "unset colorbox" << std::endl
            << "splot x with pm3d, '-' using 1:2:3 with linespoints notitle pt 7 ps 0 lw 3 lc bgr '#000000'" << std::endl;


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
     * reference color (in [0,6]). Let us note i the index of the bgr pixel tested, the test returns true when |i-PARAM::colorIndex()| < PARAM::tolerance(). Last, PARAM::darkThreshold() returns the darkness threshold. Its type is the type of some bgr components.
     */
    template<typename PARAM,typename BGR>
        class Test {
            public:
                bool operator()(const BGR& val) const {
                    BGR bgr;
                    if(removeColor(val,bgr,PARAM::darkThreshold()))
                        return false;
                    return index_fit(PARAM::colorIndex(),indexOf(bgr[2],bgr[1],bgr[0]),PARAM::tolerance());
                }
        };

    template<typename TEST>
    void mask(const cv::Mat& input,
            cv::Mat& output,
            const TEST& test,
            cvBGR true_value,
            cvBGR false_value) 
    {

        input.copyTo(output);
        for(int i = 0; i < input.rows; i++)
        {
            for(int j = 0; j < input.cols; j++)
            {
                cvBGR bgr = input.at<cvBGR>(i,j);
                output.at<cvBGR>(i,j) = test(bgr) ? true_value : false_value;
            }
        }
    }



}
