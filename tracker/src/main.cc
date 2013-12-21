#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include "filter.h"
#include "dyeFilter.h"
#include "backgroundFilter.h"

using namespace std;

int main(int argc, char** argv)
{
    if(argc != 2) {
        std::cout << "Usage : " << argv[0] << " <color-ref in [0,6]>" << std::endl;
        return 0;
    }

    cv::VideoCapture cap(-1); // open the default camera
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 800); 
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 600); 
    if(!cap.isOpened())  // check if we succeeded
        return -1;


    vector<GR::ImageFilter*> filters;
    GR::BackgroundFilter *bgFilter = new GR::BackgroundFilter(5);
    filters.push_back(new GR::DyeFilter(atof(argv[1]), 0.7, 20));
    filters.push_back(bgFilter);

    cv::Mat output;
    cv::namedWindow("motion/color filtered",CV_WINDOW_NORMAL);
    cv::namedWindow("original",CV_WINDOW_NORMAL);
    for(;;)
    {
        cv::Mat input;
        cap >> input; // get a new frame from camera

        cv::imshow("original", input);

        for(vector<GR::ImageFilter*>::iterator it = filters.begin(); it != filters.end(); ++it) {
            (*it)->process(input, output); 

            if(it!=filters.end()){
                input = output;
            }
        }

        cv::imshow("color filtered", output);

        int c = 0;
        c = cvWaitKey( 30 );
        switch (c){
            case 's': 
                {
                    std::vector<int> compression_params;
                    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
                    compression_params.push_back(9);
                    imwrite("test.png", output, compression_params);
                    break;
                }
            case 27:
                return 0;
            case 'u':
                {
                    bgFilter->setUpdate(!bgFilter->getUpdate());
                }
            default:
                break;
        }
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
