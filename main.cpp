#include "AugReality.h"


//#define DEBUG_MSGES
//#define DEBUG_IMAGES

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
    { cout<< "Error reading marker image! " << endl; return -1; }

    string XMLfile = string(argv[2]);
    CameraCalibration CameraCalib(XMLfile);

    // Create an object of class AugReality that encompasses the entire pipeline
    AugReality AR(marker, CameraCalib);

    Mat frame;
    VideoCapture capture(0);
    if (!capture.isOpened())
    {
        cout << "Null Capture from Camera!\n";
        return 0;
    }

    for (;;)
    {
        capture >> frame;
        if (!frame.empty())
        {
            // Extract features from current frame to perform AR
            AR.processFrame(frame);

            // Render the chosen 3D object on the current frame's detected marker
            AR.render3DObject(frame);

        }
        else
        {
            cout << "No captured frame -- Break!";
            break;
        }

        if (cvWaitKey(10) == 27)  // ESC to exit
            break;
    }

    waitKey(0);

    return 0;
}

