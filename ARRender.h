#ifndef ARRENDER_H
#define ARRENDER_H

#include "CameraCalibration.h"

class ARRender
{
private:
	CameraCalibration CameraCalib;
	Mat projectionMatrix;
public:
    ARRender(const CameraCalibration& CameraCalib);
	void findProjectionMatrix(int width, int height);
};

#endif // ARRENDER_H
