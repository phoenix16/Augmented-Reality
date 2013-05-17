#include "AR.h"

//#define DEBUG_MSGES

int main(int argc, char* argv[])
{
    if(argc != 3 )
    {
        cout << "Usage: ./AR [marker image] [Camera Calibration XML file]\n" << endl;
        return 0;
    }

    // Load marker image in grayscale
    Mat marker = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE );
    if( !marker.data)
    { cerr << "Error reading marker image! " << endl; return -1; }

    string XMLfile = string(argv[2]);
    CameraCalibration cameraCalib(XMLfile);
	
	// Set window size = Camera resolution
	cv::Size windowSize;
	windowSize.width = 840;
	windowSize.height = 480;

    AR ARobj(marker, "Augmented Reality", windowSize, cameraCalib);

    Mat frame;
    VideoCapture capture(0);
    if (!capture.isOpened())
    {
        cout << "Null Capture from Camera!" << endl;
        return 0;
    }

    for (;;)
    {
        capture >> frame;
        if (!frame.empty())
        {
            ARobj.augmentReality(frame);
        }
        else
        {
            cout << "No captured frame -- Break!";
            break;
        }

        if (waitKey(10) == 27)  // ESC to exit
            break;
    }

    waitKey(0);
    return 0;
}

