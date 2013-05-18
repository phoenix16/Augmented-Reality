#ifndef ARRENDER_H
#define ARRENDER_H

#include <opencv2/highgui/highgui.hpp>
#include "CameraCalibration.h"

// Include libraries for OpenGL
//#include <windows.h>  // required if running on Windows
//#include <GL.h>
//#include <GLU.h>
//#include <glut.h>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

union Mat44
{
	float data[16];
    float mat[4][4];
};

// Friend function that updates draw callback
void ARRenderDrawCallback(void* param);

class Overlay
{
public:
  Overlay(std::string windowName, cv::Size windowSize, const CameraCalibration& c);
  ~Overlay();

  void setMarkerFound(const bool markerState);
  void setMarkerPose(const Mat pose);

  // Update background with current frame
  void updateBackground(const cv::Mat& frame);
  
  // Update window after 3D object is rendered
  void updateWindow();

private:
  bool               m_isTextureInitialized;
  unsigned int       m_backgroundTextureId;
  CameraCalibration  cameraCalib;
  cv::Mat            m_backgroundImage;
  std::string        m_windowName;
    
  bool markerFound;
  cv::Mat markerPose;

  friend void OverlayDrawCallback(void* param);

  // Render entire scene in the OpenGl window
  void draw();

  // Draws the background with video
  void drawCameraFrame();

  // Draws the AR
  void drawAugmentedScene();

  // Builds the right projection matrix from the camera calibration for AR
  void buildProjectionMatrix(const CameraCalibration& calibration, int w, int h, Mat44& result);
  
  // Draws the coordinate axis 
  void drawCoordinateAxis();
  
  // Draw the cube model
  void drawCubeModel();

};



#endif // ARRENDER_H
