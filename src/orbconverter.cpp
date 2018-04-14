/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*
*
*Adapted for OpenDLV in 2018
*/


#include "orbconverter.hpp"

std::vector<cv::Mat> Orbconverter::toDescriptorVector(const cv::Mat &Descriptors)
{
    std::vector<cv::Mat> descriptorVector;
    descriptorVector.reserve(Descriptors.rows);
    for (int j=0;j<Descriptors.rows;j++)
        descriptorVector.push_back(Descriptors.row(j));

    return descriptorVector;
}

g2o::SE3Quat Orbconverter::toSE3Quat(const cv::Mat &cvMat)
{
    Eigen::Matrix3d R;
    R << cvMat.at<float>(0,0), cvMat.at<float>(0,1), cvMat.at<float>(0,2),
         cvMat.at<float>(1,0), cvMat.at<float>(1,1), cvMat.at<float>(1,2),
         cvMat.at<float>(2,0), cvMat.at<float>(2,1), cvMat.at<float>(2,2);
    Eigen::Vector3d t(cvMat.at<float>(0,3), cvMat.at<float>(1,3), cvMat.at<float>(2,3));

    return g2o::SE3Quat(R,t);
}

cv::Mat Orbconverter::toCvMat(const g2o::SE3Quat &g2oSE3)
{
    Eigen::Matrix<double,4,4> eigMat = g2oSE3.to_homogeneous_matrix();
    return toCvMat(eigMat);
}

cv::Mat Orbconverter::toCvMat(const g2o::Sim3 &g2oSim3)
{
    Eigen::Matrix3d eigR = g2oSim3.rotation().toRotationMatrix();
    Eigen::Vector3d eigt = g2oSim3.translation();
    double s = g2oSim3.scale();
    return toCvSE3(s*eigR,eigt);
}

cv::Mat Orbconverter::toCvMat(const Eigen::Matrix<double,4,4> &matrix)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=static_cast<float>(matrix(i,j));

    return cvMat.clone();
}

cv::Mat Orbconverter::toCvMat(const Eigen::Matrix3d &matrix)
{
    cv::Mat cvMat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=static_cast<float>(matrix(i,j));

    return cvMat.clone();
}

cv::Mat Orbconverter::toCvMat(const Eigen::Vector3d &vector)
{
    cv::Mat cvMat(3,1,CV_32F);
    for(int i=0;i<3;i++)
            cvMat.at<float>(i)=static_cast<float>(vector(i));

    return cvMat.clone();
}

cv::Mat Orbconverter::toCvSE3(const Eigen::Matrix3d &R, const Eigen::Vector3d &t)
{
    cv::Mat cvMat = cv::Mat::eye(4,4,CV_32F);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            cvMat.at<float>(i,j)=static_cast<float>(R(i,j));
        }
    }
    for(int i=0;i<3;i++)
    {
        cvMat.at<float>(i,3)=static_cast<float>(t(i));
    }

    return cvMat.clone();
}

Eigen::Vector3d Orbconverter::toVector3d(const cv::Mat &cvVector)
{
    Eigen::Vector3d vector;
    vector << cvVector.at<double>(0), cvVector.at<double>(1), cvVector.at<double>(2);

    return vector;
}

Eigen::Vector3d Orbconverter::toVector3d(const cv::Point3f &cvPoint)
{
    Eigen::Vector3d vector;
    vector << cvPoint.x, cvPoint.y, cvPoint.z;

    return vector;
}

Eigen::Matrix3d Orbconverter::toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix3d matrix;

    matrix << cvMat3.at<double>(0,0), cvMat3.at<double>(0,1), cvMat3.at<double>(0,2),
         cvMat3.at<double>(1,0), cvMat3.at<double>(1,1), cvMat3.at<double>(1,2),
         cvMat3.at<double>(2,0), cvMat3.at<double>(2,1), cvMat3.at<double>(2,2);

    return matrix;
}

std::vector<float> Orbconverter::toQuaternion(const cv::Mat &matrix)
{
    Eigen::Matrix3d eigMat = toMatrix3d(matrix);
    Eigen::Quaterniond quaternion(eigMat);

    std::vector<float> vector(4);
    vector[0] = static_cast<float>(quaternion.x());
    vector[1] = static_cast<float>(quaternion.y());
    vector[2] = static_cast<float>(quaternion.z());
    vector[3] = static_cast<float>(quaternion.w());

return vector;
}

