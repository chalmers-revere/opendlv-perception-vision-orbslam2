/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* Modified for use within the OpenDLV framework by Marcus Andersson, Martin Baerveldt, Linus Eiderström Swahn and Pontus Pohl
* Copyright (C) 2018 Chalmers Revere
* For more information see <https://github.com/chalmers-revere/opendlv-perception-vision-orbslam2>
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
*/

#ifndef SIM3SOLVER_HPP
#define SIM3SOLVER_HPP

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "orbframe.hpp"

#include <opencv2/opencv.hpp>
#include <vector>

class Sim3Solver {
 public:
  Sim3Solver();
  ~Sim3Solver();

  Sim3Solver(std::shared_ptr<OrbKeyFrame> localKeyFrame1, std::shared_ptr<OrbKeyFrame> localKeyFrame2, const std::vector<std::shared_ptr<OrbMapPoint>> &matchedMapPoints, const bool bFixScale = true);

  void SetRansacParameters(double probability = 0.99, int minInliers = 6 , int maxIterations = 300);

  cv::Mat find(std::vector<bool> &vbInliers12, int &nInliers);

  cv::Mat iterate(int nIterations, bool &bNoMore, std::vector<bool> &vbInliers, int &nInliers);

  cv::Mat GetEstimatedRotation();

  cv::Mat GetEstimatedTranslation();

  float GetEstimatedScale();

  int randomInt(int max, int min);


 protected:

  void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C);

  void ComputeSim3(cv::Mat &P1, cv::Mat &P2);

  void CheckInliers();

  void Project(const std::vector<cv::Mat> &vP3Dw, std::vector<cv::Mat> &vP2D, cv::Mat Tcw, cv::Mat K);
  
  void FromCameraToImage(const std::vector<cv::Mat> &vP3Dc, std::vector<cv::Mat> &vP2D, cv::Mat K);

 protected:

    // KeyFrames and matches
    std::shared_ptr<OrbKeyFrame> m_keyFrame1 = {};
    std::shared_ptr<OrbKeyFrame> m_keyFrame2= {};

    std::vector<cv::Mat> mvX3Dc1 = {};
    std::vector<cv::Mat> mvX3Dc2 = {};
    std::vector<std::shared_ptr<OrbMapPoint>> m_mapPoints1 = {};
    std::vector<std::shared_ptr<OrbMapPoint>> m_mapPoints2 = {};
    std::vector<std::shared_ptr<OrbMapPoint>> m_matchedMapPoints = {};
    std::vector<size_t> mvnIndices1 = {};
    std::vector<size_t> m_SigmaSquare1 = {};
    std::vector<size_t> m_SigmaSquare2 = {};
    std::vector<size_t> m_maxError1 = {};
    std::vector<size_t> m_maxError2 = {};

    int Ni = {};
    int m_sizeMatchedMP = {};

    // Current Estimation
    cv::Mat mR12i = {};
    cv::Mat mt12i = {};
    float ms12i = {};
    cv::Mat mT12i = {};
    cv::Mat mT21i = {};
    std::vector<bool> mvbInliersi = {};
    int mnInliersi = {};

    // Current Ransac State
    int mnIterations;
    std::vector<bool> mvbBestInliers = {};
    int mnBestInliers;
    cv::Mat mBestT12 = {};
    cv::Mat mBestRotation = {};
    cv::Mat mBestTranslation = {};
    float mBestScale = {};

    // Scale is fixed to 1 in the stereo/RGBD case
    bool mbFixScale = {};

    // Indices for random selection
    std::vector<size_t> mvAllIndices = {};

    // Projections
    std::vector<cv::Mat> mvP1im1 = {};
    std::vector<cv::Mat> mvP2im2 = {};

    // RANSAC probability
    double mRansacProb = {};

    // RANSAC min inliers
    int mRansacMinInliers = {};

    // RANSAC max iterations
    int mRansacMaxIts = {};

    // Threshold inlier/outlier. e = dist(Pi,T_ij*Pj)^2 < 5.991*mSigma2
    float mTh = {};
    float mSigma2 = {};

    // Calibration
    cv::Mat m_calibrationK1 = {};
	  cv::Mat m_calibrationK2 = {};
};


#endif //ORBMAP_HPP