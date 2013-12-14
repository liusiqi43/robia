#include <opencv2/opencv.hpp>
#include "filter.h"
#include "dye.h"

namespace gesReg{

    class TestParam {
        public:
            static double        reference;
            static double        colorIndex(void) {return reference;}
            static double        tolerance(void)  {return .3;}
            static unsigned char darkThreshold(void){return 50;}
    };


    typedef dye::Test<TestParam, dye::cvBGR> Test;


    class dyeFilter: public ImageFilter {
        private:
        public:
            void process(const cv::Mat&, cv::Mat&);
    };
}
