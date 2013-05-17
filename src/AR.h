#ifndef AR_H
#define AR_H

#include "MarkerDetector.h"
#include "Overlay.h"

class AR
{
private:
    MarkerDetector markerObj;
    CameraCalibration cameraCalib;
	Overlay renderObj; 	// its constructor updates OpenGL draw callback

	bool markerFound;
    Mat markerPose;	

	bool processFrame(cv::Mat& frame);

public:
    AR(Mat& marker, std::string windowName, cv::Size windowSize, CameraCalibration& CameraCalib);

    void augmentReality(const Mat& cameraFrame);
};



#endif // AR_H
