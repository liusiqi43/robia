// Seulement fait #include une fois. En particulier, c'est pour les fichiers .h
#pragma once

#include <opencv2/opencv.hpp>
#include <exception>
#include "dye.h"

// Gesture recognition
namespace GR{

    // Faire une excepetion d'un programme
    class ColorIndexOutOfRange : public std::exception {
        virtual const char* what() const throw(){
            return "Color index out of range, [0..6]";
        }
    };


    class DyeFilter {
        private:
            dye::DyeParams params;

        public:
            DyeFilter(double ref, 
                    double tolerance=.3, 
                    unsigned char darkThreshold=50); 
            void process( const cv::Mat& src, cv::Mat& output, std::vector<cv::Point2d>& points);
    };
}
