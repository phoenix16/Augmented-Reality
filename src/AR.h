#ifndef AR_H
#define AR_H

#include "MarkerDetector.h"
#include "Overlay.h"

class AR
{
private:
    MarkerDetector markerObj;
    CameraCalibration cameraCalib;
    Overlay renderObj;

public:
    AR(Mat& marker, std::string windowName, cv::Size windowSize, CameraCalibration& cameraCalib);

    void augmentReality(const Mat& cameraFrame);
};



#endif // AR_H
