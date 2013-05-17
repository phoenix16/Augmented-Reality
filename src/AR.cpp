#include "AR.h"

AR::AR(Mat& marker, std::string windowName, cv::Size windowSize, CameraCalibration& CameraCalib)
    : markerObj(marker),
	  cameraCalib(CameraCalib),
	  renderObj(windowName, windowSize, CameraCalib)
{
}

// Public function the performs the entire AR pipeline
void AR::augmentReality(const Mat& cameraFrame)
{	    
	// Clone image used for background (to draw overlay on it)
    cv::Mat frame = cameraFrame.clone();

	// Find the marker in current frame and estimate its pose if found
	processFrame(frame);
	
    // Update overlay background with current frame
    renderObj.updateBackground(frame);

	// Set the state of the marker (found in frame or not)
	// so that 3D object is overlaid only when marker is found
	renderObj.setMarkerFound(markerObj.findMarkerInFrame(frame));

	// Get the marker pose (will be populated only if marker is found)
	renderObj.setMarkerPose(markerObj.getPose());

    // Request redraw of the window:
    renderObj.updateWindow();
}

// Private function to find the marker in current frame and estimate its pose if found
bool AR::processFrame(cv::Mat& frame)
{
	bool markerFound = false;
	if (markerObj.findMarkerInFrame(frame))
	{
		markerFound = true;
		// If marker is found, estimate its pose
	}
	return markerFound;
}
