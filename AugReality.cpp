#include "AugReality.h"

AugReality::AugReality(Mat& marker, CameraCalibration& CameraCalib)
    : m(marker),
      CameraCalib(CameraCalib)
{
}


// Extract features from stored marker
void AugReality::processMarker()
{
    m.trainMatcher();
}

// Extract features from current frame and detect marker in it
void AugReality::processFrame(Mat& frame)
{
    bool markerFound = m.findMarkerInFrame(frame, markerCornersInFrame_2D);

    if (markerFound)
    {

        m.findMarkerLocation(CameraCalib, markerCornersInFrame_2D, pose);
    }
}


// Render the chosen 3D object on the current frame's detected marker
void AugReality::render3DObject(Mat& frame)
// Mat& pose, vector<Point2f>& markerCornersInFrame_2D)
{
}
