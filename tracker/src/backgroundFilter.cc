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
#include <queue>
#include "backgroundFilter.h"

bool largerThan (const std::vector<cv::Point>& contour1, const std::vector<cv::Point>& contour2)
{
    double area1 = fabs(contourArea(cv::Mat(contour1)));
    double area2 = fabs(contourArea(cv::Mat(contour2)));
	return area1 > area2;	
}

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

    std::sort(contours.begin(), contours.end(), largerThan); 
   
    int count = 0;
    cv::Scalar color(255, 255, 255);
    while(count < this->m_iMaxContour && count < contours.size()){
        cv::drawContours( dst, contours, count, color, CV_FILLED, 8, hierarchy );
        ++count;
    }
}

GR::BackgroundFilter::BackgroundFilter(int maxContour, bool update, int noise)
    :noise(noise), m_bUpdateBgModel(update), m_iMaxContour(maxContour) {
        this->m_BgSubtractor.set("noiseSigma", this->noise);
    }

void GR::BackgroundFilter::process(const cv::Mat& in, cv::Mat& out){
    this->m_BgSubtractor(in, this->m_aBgMask, this->m_bUpdateBgModel ? -1 : 0);
    refineSegments(in, this->m_aBgMask, out);

    bitwise_and(in, out, out);
}
