/*===============================================
 * Copyright (C) 2013 All rights reserved.
 * 
 * filename:backgroundFilter.cpp
 * author: Siqi Liu (me@siqi.fr)
 * create date:2013.12.21
 * description: 
 * 
 *===============================================*/
#include <opencv/cv.h>
#include <opencv2/video/background_segm.hpp>
#include <vector>
#include "backgroundFilter.h"

void GR::BackgroundFilter::refineSegments(const cv::Mat& img, cv::Mat& mask, cv::Mat& dst)
{
    int niters = 3;

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::Mat temp;

    dilate(mask, temp, cv::Mat(), cv::Point(-1,-1), niters);
    erode(temp, temp, cv::Mat(), cv::Point(-1,-1), niters*2);
    dilate(temp, temp, cv::Mat(), cv::Point(-1,-1), niters);

    findContours( temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

    dst = cv::Mat::zeros(img.size(), CV_8UC3);

    if( contours.size() == 0 )
        return;

    // iterate through all the top-level contours,
    // draw each connected component with its own random color
    int idx = 0, largestComp = 0;
    double maxArea = 0;

    for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
        const std::vector<cv::Point>& c = contours[idx];
        double area = fabs(contourArea(cv::Mat(c)));
        if( area > maxArea )
        {
            maxArea = area;
            largestComp = idx;
        }
    }
    cv::Scalar color( 0, 0, 255 );
    cv::drawContours( dst, contours, largestComp, color, CV_FILLED, 8, hierarchy );
}

GR::BackgroundFilter::BackgroundFilter(bool update, int noise)
    :noise(noise), m_bUpdateBgModel(update) {
        this->m_BgSubtractor.set("noiseSigma", this->noise);
    }

void GR::BackgroundFilter::process(const cv::Mat& in, cv::Mat& out){
    this->m_BgSubtractor(in, this->m_aBgMask, this->m_bUpdateBgModel ? -1 : 0);
    refineSegments(in, this->m_aBgMask, out);
}
