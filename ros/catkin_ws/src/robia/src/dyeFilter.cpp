#include <opencv2/opencv.hpp>
#include <vector>
#include "dyeFilter.h"
#include "dye.h"

dye::DyeParams::DyeParams(double colorIndex, 
        double tolerance, unsigned char darkThreshold)
: darkThreshold(darkThreshold), tolerance(tolerance) 
{
    if(colorIndex <= 6 && colorIndex >= 0){
        this->colorIndex = colorIndex;
        dye::bgrOf(this->colorIndex, this->ref);
    } else {
        throw new GR::ColorIndexOutOfRange();
    }
}

namespace GR{

    DyeFilter::DyeFilter(double ref, double tolerance, 
            unsigned char darkThreshold) 
        :params(ref, tolerance, darkThreshold)
    {
    }

    void DyeFilter::process( const cv::Mat& src, cv::Mat& output, std::vector<cv::Point2d>& points){
        dye::cvBGR true_value = this->params.getRef();
        dye::cvBGR false_value = dye::cvBGR(0,0,0);
	
	points.clear();
        src.copyTo(output);
        for(int i = 0; i < src.rows; i++)
        {
            for(int j = 0; j < src.cols; j++)
            {
                dye::cvBGR bgr = src.at<dye::cvBGR>(i,j);

                if (params.test(bgr)) {
                    output.at<dye::cvBGR>(i,j) = true_value;
                    points.push_back(cv::Point2d(j, i));
                } else {
                    output.at<dye::cvBGR>(i,j) = false_value;
                }
            }
        }
    }
}
