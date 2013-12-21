#pragma once

#include <opencv2/opencv.hpp>
#include <exception>
#include "dye.h"
#include "filter.h"

// gesture recognition
namespace GR{

    class ColorIndexOutOfRange : public std::exception {
        virtual const char* what() const throw(){
            return "Color index out of range, [0..6]";
        }
    };


    class DyeFilter: public ImageFilter {
        private:
            dye::DyeParams params;

        public:
            DyeFilter(double ref, 
                    double tolerance=.3, 
                    unsigned char darkThreshold=50); 
            void process(const cv::Mat& in, cv::Mat& out);
    };
}
