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

#ifndef INITIALIZER_H
#define INITIALIZER_H

#include <opencv2/opencv.hpp>
#include <utility>
#include "orbframe.hpp"


// THIS IS THE INITIALIZER FOR MONOCULAR SLAM. NOT USED IN THE STEREO OR RGBD CASE.
class OrbInitializer
{
    typedef std::pair<int,int> Match;

public:

    // Fix the reference frame
    OrbInitializer(std::shared_ptr<OrbFrame> referenceFrame, float sigma = 1.0, int iterations = 200);

    // Computes in parallel a fundamental matrix and a homography
    // Selects a model and tries to recover the motion and the structure from motion
    bool Initialize(std::shared_ptr<OrbFrame> currentFrame, const std::vector<int> &vMatches12,
                    cv::Mat &R21, cv::Mat &t21, std::vector<cv::Point3f> &points3d, std::vector<bool> &triangulated);


private:

    void FindHomography(std::vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);
    void FindFundamental(std::vector<bool> &vbInliers, float &score, cv::Mat &F21);

    cv::Mat ComputeHomographyMatrix(const std::vector<cv::Point2f> &vP1, const std::vector<cv::Point2f> &vP2);
    cv::Mat ComputeFundamentalMatrix(const std::vector<cv::Point2f> &vP1, const std::vector<cv::Point2f> &vP2);

    float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, std::vector<bool> &vbMatchesInliers, float sigma);

    float CheckFundamental(const cv::Mat &F21, std::vector<bool> &vbMatchesInliers, float sigma);

    bool ReconstructF(std::vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

    bool ReconstructH(std::vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

    void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

    void Normalize(const std::vector<cv::KeyPoint> &vKeys, std::vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

    int CheckRT(const cv::Mat &R, const cv::Mat &t, const std::vector<cv::KeyPoint> &vKeys1, const std::vector<cv::KeyPoint> &vKeys2,
                       const std::vector<Match> &vMatches12, std::vector<bool> &vbInliers,
                       const cv::Mat &K, std::vector<cv::Point3f> &vP3D, float th2, std::vector<bool> &vbGood, float &parallax);

    void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);

    int randomInt(int min, int max);


    // Keypoints from Reference Frame (Frame 1)
    std::vector<cv::KeyPoint> m_referenceKeys;

    // Keypoints from Current Frame (Frame 2)
    std::vector<cv::KeyPoint> m_currentKeys;

    // Current Matches from Reference to Current
    std::vector<Match> m_matches;
    std::vector<bool> m_currentMatches;

    // Calibration
    cv::Mat m_calibration;

    // Standard Deviation and Variance
    float m_sigma, m_sigma2;

    // Ransac max iterations
    int m_maxIterations;

    // Ransac sets
    std::vector<std::vector<size_t> > m_ransacSets;   

};

#endif // INITIALIZER_H
