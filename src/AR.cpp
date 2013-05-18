#include "AR.h"

AR::AR(Mat& marker, std::string windowName, cv::Size windowSize, CameraCalibration& cameraCalib)
    : markerObj(marker)
    , cameraCalib(cameraCalib)
    , renderObj(windowName, windowSize, cameraCalib)
{
    cout << "AR::constructor " << endl;
}

// Public function the performs the entire AR pipeline
void AR::augmentReality(const Mat& cameraFrame)
{
    cout << "============================ NEW FRAME ============================== " << endl;
    cout << "Function : AR::augmentReality() " << endl;

    // Clone image used for background (to draw overlay on it)
    cv::Mat frame = cameraFrame.clone();

    // Update overlay background with current frame
    renderObj.updateBackground(frame);

    // Find the marker in frame. If found, find its pose
    if (markerObj.findMarkerInFrame(frame))
    {
        renderObj.setMarkerFound(true);
        renderObj.setMarkerPose(markerObj.estimatePose(cameraCalib));
    }
    else
    {
        renderObj.setMarkerFound(false);
    }


    // Request redraw of the window:
    renderObj.updateWindow();
}
