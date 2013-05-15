#include "AugReality.h"

//#define DEBUG_MSGES
//#define DEBUG_IMAGES

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
    Mat marker = imread("mine.jpg", CV_LOAD_IMAGE_GRAYSCALE ); // marker image

    if( !marker.data)
    { cout<< "Error reading marker image! " << endl; return -1; }

    string in = string(argv[1]);
    CameraCalibration CameraCalib(in);
    // Create an object of class AugReality that encompasses the entire pipeline

    AugReality AR(marker, CameraCalib);

    // Extract features from stored marker and ready it for AR
    AR.processMarker();

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





