#ifndef AUGREALITY_H
#define AUGREALITY_H

#include "CameraCalibration.h"
#include "MarkerDetector.h"
#include "ARRender.h"

class AugReality
{
private:
    MarkerDetector m;
    CameraCalibration CameraCalib;
    vector<Point2f> markerCornersInFrame_2D;
    Mat pose;


public:
    AugReality(Mat& marker, CameraCalibration& CameraCalib);
    void processMarker();
    void processFrame(Mat& frame);
    void render3DObject(Mat &frame);
    //, Mat &pose, vector<Point2f> &markerCornersInFrame_2D);
};

#endif // AUGREALITY_H