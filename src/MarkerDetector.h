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

    vector<KeyPoint> markerKeypoints, frameKeypoints;
    Mat markerDescriptors, frameDescriptors;

    vector<Point2f> markerCorners_2D;
    vector<Point3f> markerCorners_3D;
    vector<Point2f> markerCornersInFrame_2D;	
	Point2f centroid2D; 
	Point3f centroid3D; 

    Mat pose;
    void findMarkerCorners();
    void drawContour(Mat &frame, vector<Point2f>& points, bool markerFound);
    void findMarkerCentroids();

public:
    MarkerDetector(Mat &marker);
    bool findMarkerInFrame(Mat& colorFrame);
    Mat& estimatePose(CameraCalibration& calibratedCam);
	//const Mat& getMarkerPose() const;
};

#endif // MARKERDETECTOR_H
