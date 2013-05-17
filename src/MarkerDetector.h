#ifndef MARKERDETECTOR_H
#define MARKERDETECTOR_H

#include "CameraCalibration.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <vector>

class MarkerDetector
{
private:
    Mat marker, frame;

    Ptr<FeatureDetector> detector;
    Ptr<DescriptorExtractor> extractor;
    Ptr<DescriptorMatcher> matcher;
    FlannBasedMatcher FLANNmatcher;

    vector<DMatch> matches;
    vector<KeyPoint> markerKeypoints;
    Mat markerDescriptors;

    vector<Point2f> markerCorners_2D;
    vector<Point3f> markerCorners_3D;
	vector<Point2f> markerCornersInFrame_2D;
	Mat pose;

public:
    MarkerDetector(Mat &marker);
    void findMarkerCorners();
    void drawContour(Mat &frame, vector<Point2f>& points);
    bool findMarkerInFrame(Mat& colorFrame);
    void estimatePose(CameraCalibration& calibratedCam);
	const Mat& getPose() const;
};

#endif // MARKERDETECTOR_H