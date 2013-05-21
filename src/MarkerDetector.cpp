#include "MarkerDetector.h"

MarkerDetector::MarkerDetector(Mat &marker)
    : marker(marker)
{
	detector = new SURF(400);
    extractor = new SURF();
    matcher = new FlannBasedMatcher();     
	
	//  detector  = new cv::ORB(1000);
    //  extractor = new cv::FREAK(false, false);
    //  matcher   = new cv::BFMatcher(cv::NORM_HAMMING, true);

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
    markerCorners_2D[0] = cv::Point2f(0, 0);
    markerCorners_2D[1] = cv::Point2f(width, 0);
    markerCorners_2D[2] = cv::Point2f(width, height);
    markerCorners_2D[3] = cv::Point2f(0, height);

    // Normalized dimensions
    float maxDim = max(width,height);
    float NormWidth = width / maxDim;
    float NormHeight = height / maxDim;

    // Find 3D corners of marker to estimate camera pose later
    // 3D corners of marker lie on XY plane as its planar, so z coordinate = 0
    markerCorners_3D[0] = cv::Point3f(-NormWidth, -NormHeight, 0);
    markerCorners_3D[1] = cv::Point3f( NormWidth, -NormHeight, 0);
    markerCorners_3D[2] = cv::Point3f( NormWidth,  NormHeight, 0);
    markerCorners_3D[3] = cv::Point3f(-NormWidth,  NormHeight, 0);
}

// Main detection function that performs all steps
bool MarkerDetector::findMarkerInFrame(Mat& colorFrame)
{
    cout << "\nFinding marker in current frame..." << endl;

    bool markerFound  = false;

    // Convert frame to grayscale
    cvtColor(colorFrame, frame, CV_BGR2GRAY);
	
	vector< DMatch > matches;
    detector->detect(frame, frameKeypoints);
    extractor->compute(frame, frameKeypoints, frameDescriptors);

    cout << "\n\tNumber of Keypoints found in marker = " << markerKeypoints.size() << endl;
    cout << "\n\tNumber of Keypoints found in current frame = " << frameKeypoints.size() << endl;

    matcher->match(markerDescriptors, frameDescriptors, matches);
    cout << "\nNumber of matches = " << matches.size() << endl;

    // Localize the object
    vector<cv::Point2f> obj;
    vector<cv::Point2f> scene;

    for( size_t i = 0; i < matches.size(); i++ )
    {   // Get the keypoints from the matches
        obj.push_back( markerKeypoints[ matches[i].queryIdx ].pt );
        scene.push_back( frameKeypoints[ matches[i].trainIdx ].pt );
    }

    cout << "\nFinding Homography between marker and frame..." << endl;

    // Find Homography on the matches
    Mat H = cv::findHomography( obj, scene, CV_RANSAC );

    cout << "\nApplying Perspective Transform..." << endl;
    // Apply the perspective transform to transform the corners of marker image
    // into corners of marker in the frame
    cv::perspectiveTransform( markerCorners_2D, markerCornersInFrame_2D, H);

    double area = cv::contourArea(markerCornersInFrame_2D);
    cout << "\n\t\t\t\t\tArea of contour = " << area << endl;

    if ((4000 < area) && (area  < 100000))
    {
        markerFound = true;
    }

    // Draw a contour around the marker if found
    drawContour(colorFrame, markerCornersInFrame_2D, markerFound);

    cout << "\n\t\t\t\t\tMarker detected ?   " << markerFound << endl;

    return markerFound;
}


// Draw an outline around the marker found in the scene
void MarkerDetector::drawContour(Mat &frame, vector<cv::Point2f>& points, bool markerFound)
{
    cout << "\nDrawing contours around marker in the frame..." << endl;
    if (markerFound)
    {
        for (size_t i = 0; i < markerCornersInFrame_2D.size(); i++)
        {
            cv::line(frame, points[i], points[ (i+1) % points.size() ], Scalar(255, 0, 0), 4);
        }
    }
    cv::imshow("Marker outline in scene", frame);
	cv::moveWindow("Marker outline in scene", 800, 250);
}


void MarkerDetector::findMarkerCentroids()
{	//    A = y2 – y1
    //    B = x1 – x2
    //    C = B*y1 + A*x1
    //        det     = A1B2 – A2B1;
    //            x = (B2C1 – B1C2) / det
    //            y = (A1C2 – A2C1) / det

    float A1, B1, C1, A2, B2, C2, det;

	// Find the 2D coordinates of the centroid of the estimated marker (output of perspective transform)
    A1 = markerCornersInFrame_2D[2].y - markerCornersInFrame_2D[0].y;
    B1 = markerCornersInFrame_2D[0].x - markerCornersInFrame_2D[2].x;
    C1 = (B1*markerCornersInFrame_2D[0].y) - (A1*markerCornersInFrame_2D[0].x);

    A2 = markerCornersInFrame_2D[3].y - markerCornersInFrame_2D[1].y;
    B2 = markerCornersInFrame_2D[1].x - markerCornersInFrame_2D[3].x;
    C2 = (B2*markerCornersInFrame_2D[1].y) - (A2*markerCornersInFrame_2D[1].x);

    det = (A1*B2) - (A2*B1);
    centroid2D.x = (B2*C1) - (B1*C2) / det;
    centroid2D.y = (A1*C2) - (A2*C1) / det;

	// Find the 3D coordinates of the centroid of the source marker
	A1 = markerCorners_3D[2].y - markerCorners_3D[0].y;
    B1 = markerCorners_3D[0].x - markerCorners_3D[2].x;
    C1 = (B1*markerCorners_3D[0].y) - (A1*markerCorners_3D[0].x);

    A2 = markerCorners_3D[3].y - markerCorners_3D[1].y;
    B2 = markerCorners_3D[1].x - markerCorners_3D[3].x;
    C2 = (B2*markerCorners_3D[1].y) - (A2*markerCorners_3D[1].x);

    det = (A1*B2) - (A2*B1);
    centroid3D.x = (B2*C1) - (B1*C2) / det;
    centroid3D.y = (A1*C2) - (A2*C1) / det;
	centroid3D.z = 0; // because this point lies on XY plane
}

// Public function to find marker pose (required by OpenGL render function)
Mat& MarkerDetector::estimatePose(CameraCalibration& cameraCalib)
{
    cout << "\nEstimating Camera Pose..." << endl;
    pose.create(3, 4, CV_32F);
    Mat rVec, tVec;

    // Find centroids of detected marker and the source marker in 3D
    findMarkerCentroids();

	// Create a vector for each centroid (because solvePnP function only accepts an array of points)
	vector<cv::Point2f> centroid2D_vec;
	vector<cv::Point3f> centroid3D_vec;  
	centroid2D_vec.push_back(centroid2D);
	centroid3D_vec.push_back(centroid3D);

    // solvePnP finds camera location w.r.t to marker pose from 3D-2D point correspondences
    solvePnP(centroid3D_vec,
             centroid2D_vec,
             cameraCalib.getIntrinsicMatrix(),
             cameraCalib.getDistortionMatrix(),
             rVec,
             tVec);

    // solvePnP finds camera location w.r.t to marker pose from 3D-2D point correspondences
//    solvePnP(markerCorners_3D,
//             markerCornersInFrame_2D,
//             cameraCalib.getIntrinsicMatrix(),
//             cameraCalib.getDistortionMatrix(),
//             rVec,
//             tVec);

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

    rotationInverse.copyTo(pose.colRange(Range(0,3)));
    tVecReflected.copyTo(pose.col(3));

    cout << "\nPose matrix = " << pose << endl;
    return pose;
}

// Public Setter function
//const Mat& MarkerDetector::getMarkerPose() const 
//{
//	return pose;
//}