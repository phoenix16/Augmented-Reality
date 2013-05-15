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
    std::string XMLfile;
    Mat CameraMatrix;
    Mat DistorsionMatrix;
public:
    CameraCalibration(string InputXMLfile);
    void readCameraCalibrated_XMLFile();
    const Mat& getIntrinsicMatrix() const;
    const Mat& getDistortionMatrix() const;

};

#endif // CAMERACALIBRATION_H
