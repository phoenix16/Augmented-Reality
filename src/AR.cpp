#include "AR.h"

AR::AR(Mat& marker, std::string windowName, cv::Size windowSize, CameraCalibration& cameraCalib)
    : markerObj(marker)
    , cameraCalibObj(cameraCalib)
    , overlayObj(windowName, windowSize, cameraCalib)
{   
}

// Public function the performs the entire AR pipeline
void AR::augmentReality(const Mat& cameraFrame)
{
    cout << "============================ NEW FRAME ============================== " << endl;

    // Clone image used for background (to draw 3D object overlay on it)
    cv::Mat frame = cameraFrame.clone();

    // Update overlay background with current frame
    overlayObj.updateBackground(frame);

    // Find the marker in frame. If found, find its pose
    if (markerObj.findMarkerInFrame(frame))
    {
        overlayObj.setMarkerFound(true);
        overlayObj.setMarkerPose(markerObj.estimatePose(cameraCalibObj));
    }
    else
    {
        overlayObj.setMarkerFound(false);
    }

    // Request redraw of the window
    overlayObj.updateWindow();
}
