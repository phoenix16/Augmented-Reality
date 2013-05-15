#include "MarkerDetector.h"

MarkerDetector::MarkerDetector(Mat &marker)
    : markerObject(marker)
{
    detector  = new SURF(400);
    extractor = new SURF();
    matcher = new FlannBasedMatcher();
}

void MarkerDetector::trainMatcher()
{
    // Extract the descriptors of the marker
    detector->detect(markerObject.marker, markerKeypoints);
    Mat markerDescriptors;
    extractor->compute(markerObject.marker, markerKeypoints, markerDescriptors);

    // To enable multiple marker detection, train the matcher with a vector of marker descriptors
    matcher->clear();  // Clear any old train data in it
    vector<Mat> descriptorsVector;
    descriptorsVector.push_back(markerDescriptors);
    matcher->add(descriptorsVector);
    matcher->train();
}

// Main detection function that performs all steps
bool MarkerDetector::findMarkerInFrame(Mat& frame, vector<Point2f>& markerCornersInFrame_2D)
{
    // Convert marker to grayscale
    cvtColor(frame, frame, CV_BGRA2GRAY);

    vector<KeyPoint> frameKeypoints;
    Mat frameDescriptors;
    detector->detect(frame, frameKeypoints);
    extractor->compute(frame, frameKeypoints, frameDescriptors);

    // Find matches between current frame and marker (matcher already trained with marker)
    matcher->match(frameDescriptors, matches);

    int minMatchesAllowed = 8;
    if (matches.size() < minMatchesAllowed)
        return false;

    // Find the keypoints of the marker and frame that correspond to the best matches
    vector<Point2f> objPoints(matches.size());
    vector<Point2f> scenePoints(matches.size());

    for (size_t i = 0; i < matches.size(); i++)
    {
        objPoints[i] = markerKeypoints[matches[i].trainIdx].pt;
        scenePoints[i] = frameKeypoints[matches[i].queryIdx].pt;
    }

    // Find Homography on the best matches
    Mat homography = findHomography(objPoints, scenePoints, CV_RANSAC);

    // Apply the perspective transform obtained from homography to transform
    // the corners of marker image to obtain corners of marker in the frame
    perspectiveTransform(markerObject.markerCorners_2D, markerCornersInFrame_2D, homography);

    return matches.size() > minMatchesAllowed;
}



void MarkerDetector::findMarkerLocation(CameraCalibration calibratedCam, vector<Point2f>& markerCornersInFrame_2D, Mat& pose)
{
    Mat rotationVec, translationVec;
    Mat rotationInverse, translationVecReflected;

    // solvePnP finds camera location w.r.t to marker pose from 3D-2D point correspondences
    solvePnP(markerObject.markerCorners_3D,
             markerCornersInFrame_2D,
             calibratedCam.getIntrinsicMatrix(),
             calibratedCam.getDistortionMatrix(),
             rotationVec,
             translationVec);

    rotationVec.convertTo(rotationVec,CV_32F);
    translationVec.convertTo(translationVec ,CV_32F);

    Mat_<float> rotationMatrix(3,3);
    // Rodrigues converts a rotation vector to rotation matrix and vice-versa
    Rodrigues(rotationVec, rotationMatrix);

    // Camera Extrinsic Matrix = [R | t] where R = Rotation Matrix, t = Translation Vector
    // This represents the camera location w.r.t to marker pose.
    // We need marker pose w.r.t to the camera, so invert the the transformation

    // Rotation matrices are orthogonal => inverse = transpose
    rotationInverse = rotationMatrix.t();

    // Find the reflection of translation vector
    translationVecReflected = -translationVec;

    rotationInverse.copyTo(pose.colRange(Range(0,3)));
    translationVecReflected.copyTo(pose.col(3));
}

