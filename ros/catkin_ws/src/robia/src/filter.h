#pragma once

#include <opencv2/opencv.hpp>

namespace GR{
    class ImageFilter{
        private:

        public:
            virtual void process(const cv::Mat& src, cv::Mat& out) = 0;
    };
}
