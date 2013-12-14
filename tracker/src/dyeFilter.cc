#include <opencv2/opencv.hpp>
#include "dyeFilter.h"
#include "dye.h"

namespace gesReg{


    void dyeFilter::process( const cv::Mat& src, cv::Mat& output ){
        cv::resize(output, output, src.size());

        dye::index(src, output, DARK_RGB_COMPONENT);
    }
}
