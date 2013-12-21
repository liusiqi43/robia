/*===============================================
* Copyright (C) 2013 All rights reserved.
* 
* filename:backgroundFilter.h
* author: Siqi Liu (me@siqi.fr)
* create date:2013.12.21
* description: 
* 
*===============================================*/
#pragma once

#include <opencv/cv.h>
#include "filter.h"

namespace GR {
    class BackgroundFilter : public ImageFilter {
        private:
            bool m_bUpdateBgModel;
            int noise;
            cv::Mat m_aBgMask;
            cv::BackgroundSubtractorMOG m_BgSubtractor;

            void refineSegments(const cv::Mat& img, cv::Mat& mask, cv::Mat& dst);
        public:
            BackgroundFilter(bool updateBgModel = true, int noise = 10);
            void process(const cv::Mat& in, cv::Mat& out);
    };
} /* namespace GR */
