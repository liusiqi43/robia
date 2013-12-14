#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include "dyeFilter.h"

class TestParam {
        public:
            static double        reference;
            static double        colorIndex(void) {return reference;}
            static double        tolerance(void)  {return .3;}
            static unsigned char darkThreshold(void){return 50;}
};

double TestParam::reference = 0;

typedef dye::Test<TestParam, dye::cvBGR> Test;


int main(int argc, char** argv)
{
    if(argc != 2) {
        std::cout << "Usage : " << argv[0] << " <color-ref in [0,6]>" << std::endl;
        return 0;
    }

    cv::VideoCapture cap(-1); // open the default camera
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280); 
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720); 
    if(!cap.isOpened())  // check if we succeeded
        return -1;


    TestParam::reference = atof(argv[1]);
    dye::cvBGR ref;
    
    dye::bgrOf(TestParam::reference, ref);


    cv::Mat display;
    cv::namedWindow("frame",CV_WINDOW_NORMAL);
    for(;;)
    {
        cv::Mat frame;
        cap >> frame; // get a new frame from camera

        dye::mask(frame, display, Test(), ref, dye::cvBGR(0,0,0));
        
        cv::imshow("frame", display);

        int c = 0;
        c = cvWaitKey( 30 );
        switch (c){
            case 's': 
                {
                    std::vector<int> compression_params;
                    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
                    compression_params.push_back(9);
                    imwrite("test.png", display, compression_params);
                    break;
                }
            case 27:
                return 0;
            default:
                break;
        }
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
