#include "ARRender.h"

ARRender::ARRender(const CameraCalibration& CameraCalib)
	: CameraCalib(CameraCalib)
{
}

void ARRender::findProjectionMatrix(int width, int height)
{
//  float nearPlane = 0.01f;  // Near clipping distance
//  float farPlane  = 100.0f;  // Far clipping distance

//  // Camera parameters
//  float fx = CameraCalib.getfx(); // Focal length in x axis
//  float fy = CameraCalib.getfy(); // Focal length in y axis
//  float cx = CameraCalib.getcx(); // Camera primary point x
//  float cy = CameraCalib.getcy(); // Camera primary point y
  
//  projectionMatrix.data[0] = -2.0f * fx / width;
//  projectionMatrix.data[1] = 0.0f;
//  projectionMatrix.data[2] = 0.0f;
//  projectionMatrix.data[3] = 0.0f;

//  projectionMatrix.data[4] = 0.0f;
//  projectionMatrix.data[5] = 2.0f * fy / height;
//  projectionMatrix.data[6] = 0.0f;
//  projectionMatrix.data[7] = 0.0f;

//  projectionMatrix.data[8] = 2.0f * cx / width - 1.0f;
//  projectionMatrix.data[9] = 2.0f * cy / height - 1.0f;
//  projectionMatrix.data[10] = -( farPlane + nearPlane) / ( farPlane - nearPlane );
//  projectionMatrix.data[11] = -1.0f;

//  projectionMatrix.data[12] = 0.0f;
//  projectionMatrix.data[13] = 0.0f;
//  projectionMatrix.data[14] = -2.0f * farPlane * nearPlane / ( farPlane - nearPlane );
//  projectionMatrix.data[15] = 0.0f;
}
