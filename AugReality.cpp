#include "AugReality.h"

AugReality::AugReality(Mat& marker, CameraCalibration& CameraCalib)
    : m(marker),
      CameraCalib(CameraCalib)
{
}

// Extract features from current frame and detect marker in it
void AugReality::processFrame(Mat& frame)
{
    bool markerFound = m.findMarkerInFrame(frame);

    if (markerFound)
    {
        m.estimatePose(CameraCalib);
        cameraPose = m.getPose();
    }
}


// Render the chosen 3D object on the current frame's detected marker
void AugReality::render3DObject(Mat& frame)
{
}
