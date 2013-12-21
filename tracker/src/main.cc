#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include "filter.h"
#include "dyeFilter.h"

using namespace std;

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


    vector<GR::ImageFilter*> filters;
    filters.push_back(new GR::DyeFilter(atof(argv[1])));

    cv::Mat output;
    cv::namedWindow("color filtered",CV_WINDOW_NORMAL);
    for(;;)
    {
        cv::Mat input;
        cap >> input; // get a new frame from camera

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
            default:
                break;
        }
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
