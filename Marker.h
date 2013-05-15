#ifndef MARKER_H
#define MARKER_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

struct Marker
{
    Mat marker;
    vector<Point2f> markerCorners_2D;
    vector<Point3f> markerCorners_3D;

    Marker(Mat &marker);
    void findMarkerCorners();
    void drawContour(Mat& image, vector<Point2f> points);
};

#endif // MARKER_H
