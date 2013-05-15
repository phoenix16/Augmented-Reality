#include "CameraCalibration.h"

CameraCalibration::CameraCalibration(string InputXMLfile)
    :XMLfile(InputXMLfile)
{
}

const Mat& CameraCalibration::getIntrinsicMatrix() const
{
    return CameraMatrix;
}

const Mat& CameraCalibration::getDistortionMatrix() const
{
    return DistorsionMatrix;
}

void CameraCalibration::readCameraCalibrated_XMLFile()
{
    FileStorage fs(XMLfile, FileStorage::READ);

    int w = -1, h = -1;
    Mat DistorsionMat;

    fs["image_Width"] >> w;
    fs["image_Height"] >> h;
    fs["Distortion_Coefficients"] >> DistorsionMat;
    fs["Camera_Matrix"] >> CameraMatrix;

    if (CameraMatrix.cols == 0 || CameraMatrix.rows == 0)
    {
        cout << "Does not contains valid camera matrix" << endl;
    }
    if (w <= 0 || h <= 0)
    {
        cout << "File does not contains valid camera dimensions" << endl;
    }

    // Convert to 32bit float matrix if it is not already
    CameraMatrix.convertTo(CameraMatrix,CV_32FC1);
    DistorsionMat.convertTo(DistorsionMat,CV_32FC1);

    if (DistorsionMat.total() < 4)
    {
        cout << "Does not contains valid distortion coefficients" << endl;
    }

    // Get only the first 4 elements of Distortion matrix
    DistorsionMatrix.create(1,4,CV_32FC1);

    for (int i = 0; i < 4; i++)
        DistorsionMatrix.ptr<float>(0)[i] = DistorsionMat.ptr<float>(0)[i];

//    cout << CameraMatrix << endl;
//    cout << DistorsionMatrix << endl;

}
