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
* Adapted for Open-DLV in 2018
*/

#ifndef ORBCONVERTER_H
#define ORBCONVERTER_H

#include <opencv2/core/core.hpp>

#include <Eigen/Dense>
#include"g2o/types/sim3/types_seven_dof_expmap.h"

class Orbconverter
{
public:
    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

    static g2o::SE3Quat toSE3Quat(const cv::Mat &cvMat);
    static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &g2oSim3);

    static cv::Mat toCvMat(const g2o::SE3Quat &g2oSE3);
    static cv::Mat toCvMat(const g2o::Sim3 &g2oSim3);
    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &matrix);
    static cv::Mat toCvMat(const Eigen::Matrix3d &matrix);
    static cv::Mat toCvMat(const Eigen::Vector3d &matrix);
    static cv::Mat toCvSE3(const Eigen::Matrix3d &R, const Eigen::Vector3d &t);

    static Eigen::Vector3d toVector3d(const cv::Mat &cvVector);
    static Eigen::Vector3d toVector3d(const cv::Point3f &cvPoint);
    static Eigen::Matrix3d toMatrix3d(const cv::Mat &cvMat3);

    static std::vector<float> toQuaternion(const cv::Mat &M);
};

#endif // ORBCONVERTER_H
