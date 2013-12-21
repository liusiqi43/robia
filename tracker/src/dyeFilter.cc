#include <opencv2/opencv.hpp>
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

    void DyeFilter::process( const cv::Mat& src, cv::Mat& output ){
        dye::mask(src, output, this->params, this->params.getRef(), dye::cvBGR(0,0,0));
    }
}
