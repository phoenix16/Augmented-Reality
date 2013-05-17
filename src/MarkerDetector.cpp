#include "MarkerDetector.h"

MarkerDetector::MarkerDetector(Mat &marker)
    : marker(marker)
{
    detector  = new SURF(400);
    extractor = new SURF();
    matcher = new FlannBasedMatcher();

    // Extract the descriptors of the marker
    detector->detect(marker, markerKeypoints);
    extractor->compute(marker, markerKeypoints, markerDescriptors);

    // Find corners of marker
    findMarkerCorners();
}


void MarkerDetector::findMarkerCorners()
{
    cout << "\nFinding corners of marker..." << endl;
    const float width = (float)marker.cols;
    const float height = (float)marker.rows;

    // Alloacte memory to store the points
    markerCorners_2D.resize(4);
    markerCorners_3D.resize(4);

    // Find the 2D corners of marker
    markerCorners_2D[0] = Point2f(0, 0);
    markerCorners_2D[1] = Point2f(width, 0);
    markerCorners_2D[2] = Point2f(width, height);
    markerCorners_2D[3] = Point2f(0, height);

    // Normalized dimensions
    float maxDim = max(width,height);
    float NormWidth = width / maxDim;
    float NormHeight = height / maxDim;

    // Find 3D corners of marker to estimate camera pose later
    // 3D corners of marker lie on XY plane as its planar, so z coordinate = 0
    markerCorners_3D[0] = Point3f(-NormWidth, -NormHeight, 0);
    markerCorners_3D[1] = Point3f( NormWidth, -NormHeight, 0);
    markerCorners_3D[2] = Point3f( NormWidth,  NormHeight, 0);
    markerCorners_3D[3] = Point3f(-NormWidth,  NormHeight, 0);
}

// Main detection function that performs all steps
bool MarkerDetector::findMarkerInFrame(Mat& colorFrame)
{
    cout << "\nFinding marker in current frame..." << endl;

    // Convert frame to grayscale
    cvtColor(colorFrame, frame, CV_BGRA2GRAY);

    vector<KeyPoint> frameKeypoints;
    Mat frameDescriptors;
    detector->detect(frame, frameKeypoints);
    extractor->compute(frame, frameKeypoints, frameDescriptors);

    cout << "\n\tNumber of Keypoints found in marker = " << markerKeypoints.size() << endl;
    cout << "\n\tNumber of Keypoints found in current frame = " << frameKeypoints.size() << endl;

    // Find matches between current frame and marker
    matcher->match(markerDescriptors, frameDescriptors, matches);

    int minMatchesAllowed = 8;
    if ((int)matches.size() < minMatchesAllowed)
        return false;

   cout << "\nNumber of matches = " << matches.size() << endl;

    Mat img_matches;
    drawMatches(marker, markerKeypoints, frame, frameKeypoints,   // objImage, objKeypts, sceneImage, sceneKeypts
               matches,
               img_matches,   // drawn output sits here
               Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    // Find the keypoints of the marker and frame that correspond to the best matches
    vector<Point2f> objPoints(matches.size());
    vector<Point2f> scenePoints(matches.size());

    for (size_t i = 0; i < matches.size(); i++)
    {
        objPoints[i] = markerKeypoints[matches[i].queryIdx].pt;
        scenePoints[i] = frameKeypoints[matches[i].trainIdx].pt;
    }

    cout << "\nFinding Homography between marker and frame..." << endl;
    // Find Homography on the best matches
    Mat homography = findHomography(objPoints, scenePoints, CV_RANSAC);

    cout << "\nApplying Perspective Transform..." << endl;
    // Apply the perspective transform obtained from homography to transform
    // the corners of marker image to obtain corners of marker in the frame
    perspectiveTransform(markerCorners_2D, markerCornersInFrame_2D, homography);

    // Draw a contour around the marker found in the current frame
    drawContour(colorFrame, markerCornersInFrame_2D);

    return (int)matches.size() > minMatchesAllowed;
}


// Draw an outline around the marker found in the scene
void MarkerDetector::drawContour(Mat &frame, vector<Point2f>& points)
{
    cout << "\nDrawing contours around marker in the frame..." << endl;
      for (size_t i = 0; i < markerCornersInFrame_2D.size(); i++)
      {
        line(frame, points[i], points[ (i+1) % points.size() ], Scalar(255, 0, 0), 4);
      }
    imshow("Marker outline in scene", frame);
}


// Public function to find marker pose (required by OpenGL render function)
void MarkerDetector::estimatePose(CameraCalibration& CameraCalib)
{
    cout << "\nEstimating Camera Pose..." << endl;
	Mat pose(3, 4, CV_32F);
    Mat rVec, tVec;

    // solvePnP finds camera location w.r.t to marker pose from 3D-2D point correspondences
    solvePnP(markerCorners_3D,
             markerCornersInFrame_2D,
             CameraCalib.getIntrinsicMatrix(),
             CameraCalib.getDistortionMatrix(),
             rVec,
             tVec);

    rVec.convertTo(rVec,CV_32F);
    tVec.convertTo(tVec ,CV_32F);

    Mat_<float> rotationMatrix(3,3);
    // Rodrigues converts a rotation vector to rotation matrix and vice-versa
    Rodrigues(rVec, rotationMatrix);

    Mat rotationInverse, tVecReflected;

    // Camera Extrinsic Matrix = [R | t] where R = Rotation Matrix, t = Translation Vector
    // This represents the camera location w.r.t to marker pose.
    // We need marker pose w.r.t to the camera, so invert the the transformation

    // Rotation matrices are orthogonal => inverse = transpose
    rotationInverse = rotationMatrix.t();

    // Find the reflection of translation vector
    tVecReflected = -tVec;

    pose.create(3, 4, CV_32F);
    rotationInverse.copyTo(pose.colRange(Range(0,3)));
    tVecReflected.copyTo(pose.col(3));

    cout << "\nPose matrix = " << pose << endl;

}

const Mat& MarkerDetector::getPose() const
{
  	return pose;
}