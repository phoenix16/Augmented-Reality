#include "Marker.h"

Marker::Marker(Mat& marker)
    : marker(marker)
{
}


void Marker::findMarkerCorners()
{
    const float width = marker.cols;
    const float height = marker.rows;

    // Find the 2D corners of marker
    markerCorners_2D[0] = Point2f(0, 0);
    markerCorners_2D[1] = Point2f(width, 0);
    markerCorners_2D[2] = Point2f(width, height);
    markerCorners_2D[3] = Point2f(0, height);

    // Normalized dimensions
    const float maxDim = max(width,height);
    const float NormWidth = width / maxDim;
    const float NormHeight = height / maxDim;

    // Find 3D corners of marker
    // 3D corners of marker lie on XY plane as its planar, so z coordinate = 0
    markerCorners_3D[0] = Point3f(-NormWidth, -NormHeight, 0);
    markerCorners_3D[1] = Point3f( NormWidth, -NormHeight, 0);
    markerCorners_3D[2] = Point3f( NormWidth,  NormHeight, 0);
    markerCorners_3D[3] = Point3f(-NormWidth,  NormHeight, 0);
}



// Draw an outline around the marker found in the scene
void Marker::drawContour(Mat& image, vector<Point2f> points)
{

      for (size_t i = 0; i < points.size(); i++)
      {
        line(image, points[i], points[ (i+1) % points.size() ], Scalar(255, 0, 0), 4);
      }

//    line(image, points[0], points[1], Scalar(255, 0, 0), 4);
//    line(image, points[1], points[2], Scalar(255, 0, 0), 4);
//    line(image, points[2], points[3], Scalar(255, 0, 0), 4);
//    line(image, points[3], points[0], Scalar(255, 0, 0), 4);

    imshow( "Marker outline in scene", image );
}
