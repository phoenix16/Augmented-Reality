#ifndef MARKERDETECTOR_H
#define MARKERDETECTOR_H

#include "Marker.h"
#include "CameraCalibration.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

class MarkerDetector
{
private:
    Marker markerObject;

    Ptr<FeatureDetector> detector;
    Ptr<DescriptorExtractor> extractor;
    Ptr<DescriptorMatcher> matcher;
    vector<DMatch> matches;
    vector<KeyPoint> markerKeypoints;

public:
    MarkerDetector(Mat &marker);
    void trainMatcher();
    bool findMarkerInFrame(Mat& frame, vector<Point2f>& markerCornersInFrame_2D);
    void findMarkerLocation(CameraCalibration calibratedCam, vector<Point2f>& markerCornersInFrame_2D, Mat& pose);
};

#endif // MARKERDETECTOR_H
