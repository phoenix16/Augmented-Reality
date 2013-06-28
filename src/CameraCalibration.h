/*
 * Developer : Prakriti Chintalapoodi - c.prakriti@gmail.com 
*/

#ifndef CAMERACALIBRATION_H
#define CAMERACALIBRATION_H

#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

class CameraCalibration
{
private:
    string XMLfile;
    Mat IntrinsicMatrix;
    Mat DistorsionMatrix;
public:
    CameraCalibration(string InputXMLfile);
    void readCameraCalibrated_XMLFile();

    const Mat& getIntrinsicMatrix() const;
    const Mat& getDistortionMatrix() const;
	const float& getfx() const;
	const float& getfy() const;
	const float& getcx() const;
	const float& getcy() const;
};

#endif // CAMERACALIBRATION_H
