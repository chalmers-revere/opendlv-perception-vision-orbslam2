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
*/

#include "orbinitializer.hpp"

//#include "Thirdparty/DBoW2/DUtils/Random.h"

//#include "Optimizer.h"
//#include "ORBmatcher.h"

#include <thread>

OrbInitializer::OrbInitializer(OrbFrame &referenceFrame, float sigma, int iterations) : m_referenceKeys(),
                                                                                        m_currentKeys(),
                                                                                        m_matches(),
                                                                                        m_currentMatches(),
                                                                                        m_calibration(),
                                                                                        m_sigma(),
                                                                                        m_sigma2(),
                                                                                        m_maxIterations(),
                                                                                        m_ransacSets()
{
    //m_calibration = referenceFrame.m_calibration.clone(); We have to come up with something uniform

    m_referenceKeys = referenceFrame.GetUndistortedKeyPoints();

    m_sigma = sigma;
    m_sigma2 = sigma * sigma;
    m_maxIterations = iterations;
}

bool OrbInitializer::Initialize(OrbFrame &currentFrame, const std::vector<int> &matches, cv::Mat &R21, cv::Mat &t21,
                                std::vector<cv::Point3f> &points3d, std::vector<bool> &triangulated)
{
    // Fill structures with current keypoints and matches with reference frame
    // Reference Frame: 1, Current Frame: 2
    m_currentKeys = currentFrame.GetUndistortedKeyPoints();

    m_matches.clear();
    m_matches.reserve(m_currentKeys.size());
    m_currentMatches.resize(m_referenceKeys.size());
    for (size_t i = 0, iend = matches.size(); i < iend; i++)
    {
        if (matches[i] >= 0)
        {
            m_matches.push_back(std::make_pair(i, matches[i]));
            m_currentMatches[i] = true;
        }
        else
            m_currentMatches[i] = false;
    }

    const int N = m_matches.size();

    // Indices for minimum set selection
    std::vector<size_t> allIndices;
    allIndices.reserve(N);
    std::vector<size_t> availableIndices;

    for (int i = 0; i < N; i++)
    {
        allIndices.push_back(i);
    }

    // Generate sets of 8 points for each RANSAC iteration
    m_ransacSets = std::vector<std::vector<size_t>>(m_maxIterations, std::vector<size_t>(8, 0));

    //DUtils::Random::SeedRandOnce(0);

    for (int it = 0; it < m_maxIterations; it++)
    {
        availableIndices = allIndices;

        // Select a minimum set
        for (size_t j = 0; j < 8; j++)
        {
            int randi = 4; //DUtils::Random::RandomInt(0,availableIndices.size()-1);
            int idx = availableIndices[randi];

            m_ransacSets[it][j] = idx;

            availableIndices[randi] = availableIndices.back();
            availableIndices.pop_back();
        }
    }

    // Launch threads to compute in parallel a fundamental matrix and a homography
    std::vector<bool> homographyInlierMatches, fundamentalInlierMatches;
    float homographyScore, fundamentalScore;
    cv::Mat homography, fundamental;

    std::thread threadHomography(&OrbInitializer::FindHomography, this, std::ref(homographyInlierMatches), std::ref(homographyScore), std::ref(homography));
    std::thread threadFundamental(&OrbInitializer::FindFundamental, this, std::ref(fundamentalInlierMatches), std::ref(fundamentalScore), std::ref(fundamental));
    // Wait until both threads have finished
    threadHomography.join();
    threadFundamental.join();

    // Compute ratio of scores
    float homographyRatio = homographyScore / (homographyScore + fundamentalScore);

    // Try to reconstruct from homography or fundamental depending on the ratio (0.40-0.45)
    if (homographyRatio > 0.40)
        return ReconstructH(homographyInlierMatches, homography, m_calibration, R21, t21, points3d, triangulated, 1.0, 50);
    else //if(pF_HF>0.6)
        return ReconstructF(fundamentalInlierMatches, fundamental, m_calibration, R21, t21, points3d, triangulated, 1.0, 50);

    return false;
}

void OrbInitializer::FindHomography(std::vector<bool> &matchingInliers, float &score, cv::Mat &homographyMatrix)
{
    // Number of putative matches
    const int N = m_matches.size();

    // Normalize coordinates
    std::vector<cv::Point2f> refPoints, currentPoints;
    cv::Mat T1, T2;
    Normalize(m_referenceKeys, refPoints, T1);
    Normalize(m_currentKeys, currentPoints, T2);
    cv::Mat T2inv = T2.inv();

    // Best Results variables
    score = 0.0;
    matchingInliers = std::vector<bool>(N, false);

    // Iteration variables
    std::vector<cv::Point2f> refPointsi(8);
    std::vector<cv::Point2f> currentPointsi(8);
    cv::Mat homographyMatrixi, H12i;
    std::vector<bool> currentInliers(N, false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for (int it = 0; it < m_maxIterations; it++)
    {
        // Select a minimum set
        for (size_t j = 0; j < 8; j++)
        {
            int idx = m_ransacSets[it][j];

            refPointsi[j] = refPoints[m_matches[idx].first];
            currentPointsi[j] = currentPoints[m_matches[idx].second];
        }

        cv::Mat Hn = ComputeHomographyMatrix(refPointsi, currentPointsi);
        homographyMatrixi = T2inv * Hn * T1;
        H12i = homographyMatrixi.inv();

        currentScore = CheckHomography(homographyMatrixi, H12i, currentInliers, m_sigma);

        if (currentScore > score)
        {
            homographyMatrix = homographyMatrixi.clone();
            matchingInliers = currentInliers;
            score = currentScore;
        }
    }
}

void OrbInitializer::FindFundamental(std::vector<bool> &matchingInliers, float &score, cv::Mat &fundamentalMatrix)
{
    // Number of putative matches
    const int N = matchingInliers.size();

    // Normalize coordinates
    std::vector<cv::Point2f> refPoints, currentPoints;
    cv::Mat T1, T2;
    Normalize(m_referenceKeys, refPoints, T1);
    Normalize(m_currentKeys, currentPoints, T2);
    cv::Mat T2t = T2.t();

    // Best Results variables
    score = 0.0;
    matchingInliers = std::vector<bool>(N, false);

    // Iteration variables
    std::vector<cv::Point2f> refPointsi(8);
    std::vector<cv::Point2f> currentPointsi(8);
    cv::Mat fundamentalMatrixi;
    std::vector<bool> currentInliers(N, false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for (int it = 0; it < m_maxIterations; it++)
    {
        // Select a minimum set
        for (int j = 0; j < 8; j++)
        {
            int idx = m_ransacSets[it][j];

            refPointsi[j] = refPoints[m_matches[idx].first];
            currentPointsi[j] = currentPoints[m_matches[idx].second];
        }

        cv::Mat Fn = ComputeFundamentalMatrix(refPointsi, currentPointsi);

        fundamentalMatrixi = T2t * Fn * T1;

        currentScore = CheckFundamental(fundamentalMatrixi, currentInliers, m_sigma);

        if (currentScore > score)
        {
            fundamentalMatrix = fundamentalMatrixi.clone();
            matchingInliers = currentInliers;
            score = currentScore;
        }
    }
}

cv::Mat OrbInitializer::ComputeHomographyMatrix(const std::vector<cv::Point2f> &referencePoints, const std::vector<cv::Point2f> &currentPoints)
{
    const int N = referencePoints.size();

    cv::Mat A(2 * N, 9, CV_32F);

    for (int i = 0; i < N; i++)
    {
        const float uRef = referencePoints[i].x;
        const float vRef = referencePoints[i].y;
        const float uCurr = currentPoints[i].x;
        const float vCurr = currentPoints[i].y;

        A.at<float>(2 * i, 0) = 0.0;
        A.at<float>(2 * i, 1) = 0.0;
        A.at<float>(2 * i, 2) = 0.0;
        A.at<float>(2 * i, 3) = -uRef;
        A.at<float>(2 * i, 4) = -vRef;
        A.at<float>(2 * i, 5) = -1;
        A.at<float>(2 * i, 6) = vCurr * uRef;
        A.at<float>(2 * i, 7) = vCurr * vRef;
        A.at<float>(2 * i, 8) = vCurr;

        A.at<float>(2 * i + 1, 0) = uRef;
        A.at<float>(2 * i + 1, 1) = vRef;
        A.at<float>(2 * i + 1, 2) = 1;
        A.at<float>(2 * i + 1, 3) = 0.0;
        A.at<float>(2 * i + 1, 4) = 0.0;
        A.at<float>(2 * i + 1, 5) = 0.0;
        A.at<float>(2 * i + 1, 6) = -uCurr * uRef;
        A.at<float>(2 * i + 1, 7) = -uCurr * vRef;
        A.at<float>(2 * i + 1, 8) = -uCurr;
    }

    cv::Mat u, w, vt;

    cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    return vt.row(8).reshape(0, 3);
}

cv::Mat OrbInitializer::ComputeFundamentalMatrix(const std::vector<cv::Point2f> &referencePoints, const std::vector<cv::Point2f> &currentPoints)
{
    const int N = referencePoints.size();

    cv::Mat A(N, 9, CV_32F);

    for (int i = 0; i < N; i++)
    {
        const float uRef = referencePoints[i].x;
        const float vRef = referencePoints[i].y;
        const float uCurr = currentPoints[i].x;
        const float vCurr = currentPoints[i].y;

        A.at<float>(i, 0) = uCurr * uRef;
        A.at<float>(i, 1) = uCurr * vRef;
        A.at<float>(i, 2) = uCurr;
        A.at<float>(i, 3) = vCurr * uRef;
        A.at<float>(i, 4) = vCurr * vRef;
        A.at<float>(i, 5) = vCurr;
        A.at<float>(i, 6) = uRef;
        A.at<float>(i, 7) = vRef;
        A.at<float>(i, 8) = 1;
    }

    cv::Mat u, w, vt;

    cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    cv::Mat Fpre = vt.row(8).reshape(0, 3);

    cv::SVDecomp(Fpre, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    w.at<float>(2) = 0;

    return u * cv::Mat::diag(w) * vt;
}

float OrbInitializer::CheckHomography(const cv::Mat &homographyMatrix, const cv::Mat &homographyMatrixInv, std::vector<bool> &matchingInliers, float sigma)
{
    const int N = m_matches.size();

    const float h11 = homographyMatrix.at<float>(0, 0);
    const float h12 = homographyMatrix.at<float>(0, 1);
    const float h13 = homographyMatrix.at<float>(0, 2);
    const float h21 = homographyMatrix.at<float>(1, 0);
    const float h22 = homographyMatrix.at<float>(1, 1);
    const float h23 = homographyMatrix.at<float>(1, 2);
    const float h31 = homographyMatrix.at<float>(2, 0);
    const float h32 = homographyMatrix.at<float>(2, 1);
    const float h33 = homographyMatrix.at<float>(2, 2);

    const float h11inv = homographyMatrixInv.at<float>(0, 0);
    const float h12inv = homographyMatrixInv.at<float>(0, 1);
    const float h13inv = homographyMatrixInv.at<float>(0, 2);
    const float h21inv = homographyMatrixInv.at<float>(1, 0);
    const float h22inv = homographyMatrixInv.at<float>(1, 1);
    const float h23inv = homographyMatrixInv.at<float>(1, 2);
    const float h31inv = homographyMatrixInv.at<float>(2, 0);
    const float h32inv = homographyMatrixInv.at<float>(2, 1);
    const float h33inv = homographyMatrixInv.at<float>(2, 2);

    matchingInliers.resize(N);

    float score = 0;

    const float th = 5.991f;

    const float invSigmaSquare = 1.0f / (sigma * sigma);

    for (int i = 0; i < N; i++)
    {
        bool isInlier = true;

        const cv::KeyPoint &kp1 = m_referenceKeys[m_matches[i].first];
        const cv::KeyPoint &kp2 = m_currentKeys[m_matches[i].second];

        const float uRef = kp1.pt.x;
        const float vRef = kp1.pt.y;
        const float uCurr = kp2.pt.x;
        const float vCurr = kp2.pt.y;

        // Reprojection error in first image
        // x2in1 = homographyMatrixInv*x2

        const float wCurrInRefInv = 1.0f / (h31inv * uCurr + h32inv * vCurr + h33inv);
        const float uCurrInRef = (h11inv * uCurr + h12inv + vCurr * h13inv) * wCurrInRefInv;
        const float vCurrInRef = (h21inv * uCurr * h22inv + vCurr + h23inv) * wCurrInRefInv;

        const float squareDist1 = (uRef - uCurrInRef) * (uRef - uCurrInRef) + (vRef - vCurrInRef) * (vRef - vCurrInRef);

        const float chiSquare1 = squareDist1 * invSigmaSquare;

        if (chiSquare1 > th)
            isInlier = false;
        else
            score += th - chiSquare1;

        // Reprojection error in second image
        // x1in2 = homographyMatrix*x1

        const float wRefInCurrInv = 1.0f / (h31 * uRef + h32 * vRef + h33);
        const float uRefInCurr = (h11 * uRef + h12 * vRef + h13) * wRefInCurrInv;
        const float vRefInCurr = (h21 * uRef + h22 * vRef + h23) * wRefInCurrInv;

        const float squareDist2 = (uCurr - uRefInCurr) * (uCurr - uRefInCurr) + (vCurr - vRefInCurr) * (vCurr - vRefInCurr);

        const float chiSquare2 = squareDist2 * invSigmaSquare;

        if (chiSquare2 > th)
            isInlier = false;
        else
            score += th - chiSquare2;

        if (isInlier)
            matchingInliers[i] = true;
        else
            matchingInliers[i] = false;
    }

    return score;
}

float OrbInitializer::CheckFundamental(const cv::Mat &fundamentalMatrix, std::vector<bool> &matchingInliers, float sigma)
{
    const int N = m_matches.size();

    const float f11 = fundamentalMatrix.at<float>(0, 0);
    const float f12 = fundamentalMatrix.at<float>(0, 1);
    const float f13 = fundamentalMatrix.at<float>(0, 2);
    const float f21 = fundamentalMatrix.at<float>(1, 0);
    const float f22 = fundamentalMatrix.at<float>(1, 1);
    const float f23 = fundamentalMatrix.at<float>(1, 2);
    const float f31 = fundamentalMatrix.at<float>(2, 0);
    const float f32 = fundamentalMatrix.at<float>(2, 1);
    const float f33 = fundamentalMatrix.at<float>(2, 2);

    matchingInliers.resize(N);

    float score = 0;

    const float th = 3.841f;
    const float thScore = 5.991f;

    const float invSigmaSquare = 1.0f / (sigma * sigma);

    for (int i = 0; i < N; i++)
    {
        bool isInlier = true;

        const cv::KeyPoint &kp1 = m_referenceKeys[m_matches[i].first];
        const cv::KeyPoint &kp2 = m_currentKeys[m_matches[i].second];

        const float uRef = kp1.pt.x;
        const float vRef = kp1.pt.y;
        const float uCurr = kp2.pt.x;
        const float vCurr = kp2.pt.y;

        // Reprojection error in second image
        // l2=fundamentalMatrixx1=(a2,b2,c2)

        const float a2 = f11 * uRef + f12 * vRef + f13;
        const float b2 = f21 * uRef + f22 * vRef + f23;
        const float c2 = f31 * uRef + f32 * vRef + f33;

        const float num2 = a2 * uCurr + b2 * vCurr + c2;

        const float squareDist1 = num2 * num2 / (a2 * a2 + b2 * b2);

        const float chiSquare1 = squareDist1 * invSigmaSquare;

        if (chiSquare1 > th)
            isInlier = false;
        else
            score += thScore - chiSquare1;

        // Reprojection error in second image
        // l1 =x2tfundamentalMatrix=(a1,b1,c1)

        const float a1 = f11 * uCurr + f21 + vCurr + f31;
        const float b1 = f12 * uCurr + f22 + vCurr + f32;
        const float c1 = f13 * uCurr + f23 + vCurr + f33;

        const float num1 = a1 * uRef + b1 * vRef + c1;

        const float squareDist2 = num1 * num1 / (a1 * a1 + b1 * b1);

        const float chiSquare2 = squareDist2 * invSigmaSquare;

        if (chiSquare2 > th)
            isInlier = false;
        else
            score += thScore - chiSquare2;

        if (isInlier)
            matchingInliers[i] = true;
        else
            matchingInliers[i] = false;
    }

    return score;
}

bool OrbInitializer::ReconstructF(std::vector<bool> &matchingInliers, cv::Mat &fundamentalMatrix, cv::Mat &K,
                                  cv::Mat &R21, cv::Mat &t21, std::vector<cv::Point3f> &points3d, std::vector<bool> &triangulated, float minParallax, int minTriangulated)
{
    int N = 0;
    for (size_t i = 0, iend = matchingInliers.size(); i < iend; i++)
        if (matchingInliers[i])
            N++;

    // Compute Essential Matrix from Fundamental Matrix
    cv::Mat E21 = K.t() * fundamentalMatrix * K;

    cv::Mat R1, R2, t;

    // Recover the 4 motion hypotheses
    DecomposeE(E21, R1, R2, t);

    cv::Mat t1 = t;
    cv::Mat t2 = -t;

    // Reconstruct with the 4 hyphoteses and check
    std::vector<cv::Point3f> points3d1, points3d2, points3d3, points3d4;
    std::vector<bool> triangulated1, triangulated2, triangulated3, triangulated4;
    float parallax1, parallax2, parallax3, parallax4;

    int nGood1 = CheckRT(R1, t1, m_referenceKeys, m_currentKeys, m_matches, matchingInliers, K, points3d1, 4.0f * m_sigma2, triangulated1, parallax1);
    int nGood2 = CheckRT(R2, t1, m_referenceKeys, m_currentKeys, m_matches, matchingInliers, K, points3d2, 4.0f * m_sigma2, triangulated2, parallax2);
    int nGood3 = CheckRT(R1, t2, m_referenceKeys, m_currentKeys, m_matches, matchingInliers, K, points3d3, 4.0f * m_sigma2, triangulated3, parallax3);
    int nGood4 = CheckRT(R2, t2, m_referenceKeys, m_currentKeys, m_matches, matchingInliers, K, points3d4, 4.0f * m_sigma2, triangulated4, parallax4);

    int maxGood = std::max(nGood1, std::max(nGood2, std::max(nGood3, nGood4)));

    R21 = cv::Mat();
    t21 = cv::Mat();

    int nMinGood = std::max(static_cast<int>(0.9 * N), minTriangulated);

    int nsimilar = 0;
    if (nGood1 > 0.7 * maxGood)
        nsimilar++;
    if (nGood2 > 0.7 * maxGood)
        nsimilar++;
    if (nGood3 > 0.7 * maxGood)
        nsimilar++;
    if (nGood4 > 0.7 * maxGood)
        nsimilar++;

    // If there is not a clear winner or not enough triangulated points reject initialization
    if (maxGood < nMinGood || nsimilar > 1)
    {
        return false;
    }

    // If best reconstruction has enough parallax initialize
    if (maxGood == nGood1)
    {
        if (parallax1 > minParallax)
        {
            points3d = points3d1;
            triangulated = triangulated1;

            R1.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }
    else if (maxGood == nGood2)
    {
        if (parallax2 > minParallax)
        {
            points3d = points3d2;
            triangulated = triangulated2;

            R2.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }
    else if (maxGood == nGood3)
    {
        if (parallax3 > minParallax)
        {
            points3d = points3d3;
            triangulated = triangulated3;

            R1.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }
    else if (maxGood == nGood4)
    {
        if (parallax4 > minParallax)
        {
            points3d = points3d4;
            triangulated = triangulated4;

            R2.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }

    return false;
}

bool OrbInitializer::ReconstructH(std::vector<bool> &matchingInliers, cv::Mat &homographyMatrix, cv::Mat &K,
                                  cv::Mat &R21, cv::Mat &t21, std::vector<cv::Point3f> &points3d, std::vector<bool> &triangulated, float minParallax, int minTriangulated)
{
    int N = 0;
    for (size_t i = 0, iend = matchingInliers.size(); i < iend; i++)
        if (matchingInliers[i])
            N++;

    // We recover 8 motion hypotheses using the method of Faugeras et al.
    // Motion and structure from motion in a piecewise planar environment.
    // International Journal of Pattern Recognition and Artificial Intelligence, 1988

    cv::Mat invK = K.inv();
    cv::Mat A = invK * homographyMatrix * K;

    cv::Mat U, w, Vt, V;
    cv::SVD::compute(A, w, U, Vt, cv::SVD::FULL_UV);
    V = Vt.t();

    float s = static_cast<float>(cv::determinant(U) * cv::determinant(Vt));

    float d1 = w.at<float>(0);
    float d2 = w.at<float>(1);
    float d3 = w.at<float>(2);

    if (d1 / d2 < 1.00001 || d2 / d3 < 1.00001)
    {
        return false;
    }

    std::vector<cv::Mat> vR, vt, vn;
    vR.reserve(8);
    vt.reserve(8);
    vn.reserve(8);
    //n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
    float aux1 = (float)sqrt((d1 * d1 - d2 * d2) / (d1 * d1 - d3 * d3));
    float aux3 = (float)sqrt((d2 * d2 - d3 * d3) / (d1 * d1 - d3 * d3));
    float x1[] = {aux1, aux1, -aux1, -aux1};
    float x3[] = {aux3, -aux3, aux3, -aux3};

    //case d'=d2
    float aux_stheta = (float)sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 + d3) * d2);

    float ctheta = (d2 * d2 + d1 * d3) / ((d1 + d3) * d2);
    float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

    for (int i = 0; i < 4; i++)
    {
        cv::Mat Rp = cv::Mat::eye(3, 3, CV_32F);
        Rp.at<float>(0, 0) = ctheta;
        Rp.at<float>(0, 2) = -stheta[i];
        Rp.at<float>(2, 0) = stheta[i];
        Rp.at<float>(2, 2) = ctheta;

        cv::Mat R = s * U * Rp * Vt;
        vR.push_back(R);

        cv::Mat tp(3, 1, CV_32F);
        tp.at<float>(0) = x1[i];
        tp.at<float>(1) = 0;
        tp.at<float>(2) = -x3[i];
        tp *= d1 - d3;

        cv::Mat t = U * tp;
        vt.push_back(t / cv::norm(t));

        cv::Mat np(3, 1, CV_32F);
        np.at<float>(0) = x1[i];
        np.at<float>(1) = 0;
        np.at<float>(2) = x3[i];

        cv::Mat n = V * np;
        if (n.at<float>(2) < 0)
            n = -n;
        vn.push_back(n);
    }

    //case d'=-d2
    float aux_sphi = (float)sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 - d3) * d2);

    float cphi = (d1 * d3 - d2 * d2) / ((d1 - d3) * d2);
    float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

    for (int i = 0; i < 4; i++)
    {
        cv::Mat Rp = cv::Mat::eye(3, 3, CV_32F);
        Rp.at<float>(0, 0) = cphi;
        Rp.at<float>(0, 2) = sphi[i];
        Rp.at<float>(1, 1) = -1;
        Rp.at<float>(2, 0) = sphi[i];
        Rp.at<float>(2, 2) = -cphi;

        cv::Mat R = s * U * Rp * Vt;
        vR.push_back(R);

        cv::Mat tp(3, 1, CV_32F);
        tp.at<float>(0) = x1[i];
        tp.at<float>(1) = 0;
        tp.at<float>(2) = x3[i];
        tp *= d1 + d3;

        cv::Mat t = U * tp;
        vt.push_back(t / cv::norm(t));

        cv::Mat np(3, 1, CV_32F);
        np.at<float>(0) = x1[i];
        np.at<float>(1) = 0;
        np.at<float>(2) = x3[i];

        cv::Mat n = V * np;
        if (n.at<float>(2) < 0)
            n = -n;
        vn.push_back(n);
    }

    int bestGood = 0;
    int secondBestGood = 0;
    int bestSolutionIdx = -1;
    float bestParallax = -1;
    std::vector<cv::Point3f> bestP3D;
    std::vector<bool> bestTriangulated;

    // Instead of applying the visibility constraints proposed in the Faugeras' paper (which could fail for points seen with low parallax)
    // We reconstruct all hypotheses and check in terms of triangulated points and parallax
    for (size_t i = 0; i < 8; i++)
    {
        float parallaxi;
        std::vector<cv::Point3f> points3di;
        std::vector<bool> triangulatedi;
        int nGood = CheckRT(vR[i], vt[i], m_referenceKeys, m_currentKeys, m_matches, matchingInliers, K, points3di, 4.0f * m_sigma2, triangulatedi, parallaxi);

        if (nGood > bestGood)
        {
            secondBestGood = bestGood;
            bestGood = nGood;
            bestSolutionIdx = i;
            bestParallax = parallaxi;
            bestP3D = points3di;
            bestTriangulated = triangulatedi;
        }
        else if (nGood > secondBestGood)
        {
            secondBestGood = nGood;
        }
    }

    if (secondBestGood < 0.75 * bestGood && bestParallax >= minParallax && bestGood > minTriangulated && bestGood > 0.9 * N)
    {
        vR[bestSolutionIdx].copyTo(R21);
        vt[bestSolutionIdx].copyTo(t21);
        points3d = bestP3D;
        triangulated = bestTriangulated;

        return true;
    }

    return false;
}

void OrbInitializer::Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
{
    cv::Mat A(4, 4, CV_32F);

    A.row(0) = kp1.pt.x * P1.row(2) - P1.row(0);
    A.row(1) = kp1.pt.y * P1.row(2) - P1.row(1);
    A.row(2) = kp2.pt.x * P2.row(2) - P2.row(0);
    A.row(3) = kp2.pt.y * P2.row(2) - P2.row(1);

    cv::Mat u, w, vt;
    cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    x3D = vt.row(3).t();
    x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);
}

void OrbInitializer::Normalize(const std::vector<cv::KeyPoint> &vKeys, std::vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T)
{
    float meanX = 0;
    float meanY = 0;
    const int N = vKeys.size();

    vNormalizedPoints.resize(N);

    for (int i = 0; i < N; i++)
    {
        meanX += vKeys[i].pt.x;
        meanY += vKeys[i].pt.y;
    }

    meanX = meanX / N;
    meanY = meanY / N;

    float meanDevX = 0;
    float meanDevY = 0;

    for (int i = 0; i < N; i++)
    {
        vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
        vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;

        meanDevX += (float)fabs(vNormalizedPoints[i].x);
        meanDevY += (float)fabs(vNormalizedPoints[i].y);
    }

    meanDevX = meanDevX / N;
    meanDevY = meanDevY / N;

    float sX = 1.0f / meanDevX;
    float sY = 1.0f / meanDevY;

    for (int i = 0; i < N; i++)
    {
        vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
        vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
    }

    T = cv::Mat::eye(3, 3, CV_32F);
    T.at<float>(0, 0) = sX;
    T.at<float>(1, 1) = sY;
    T.at<float>(0, 2) = -meanX * sX;
    T.at<float>(1, 2) = -meanY * sY;
}

int OrbInitializer::CheckRT(const cv::Mat &R, const cv::Mat &t, const std::vector<cv::KeyPoint> &vKeys1, const std::vector<cv::KeyPoint> &vKeys2,
                            const std::vector<Match> &vMatches12, std::vector<bool> &matchingInliers,
                            const cv::Mat &K, std::vector<cv::Point3f> &points3d, float th2, std::vector<bool> &vbGood, float &parallax)
{
    // Calibration parameters
    const float fx = K.at<float>(0, 0);
    const float fy = K.at<float>(1, 1);
    const float cx = K.at<float>(0, 2);
    const float cy = K.at<float>(1, 2);

    vbGood = std::vector<bool>(vKeys1.size(), false);
    points3d.resize(vKeys1.size());

    std::vector<float> vCosParallax;
    vCosParallax.reserve(vKeys1.size());

    // Camera 1 Projection Matrix K[I|0]
    cv::Mat P1(3, 4, CV_32F, cv::Scalar(0));
    K.copyTo(P1.rowRange(0, 3).colRange(0, 3));

    cv::Mat O1 = cv::Mat::zeros(3, 1, CV_32F);

    // Camera 2 Projection Matrix K[R|t]
    cv::Mat P2(3, 4, CV_32F);
    R.copyTo(P2.rowRange(0, 3).colRange(0, 3));
    t.copyTo(P2.rowRange(0, 3).col(3));
    P2 = K * P2;

    cv::Mat O2 = -R.t() * t;

    int nGood = 0;

    for (size_t i = 0, iend = vMatches12.size(); i < iend; i++)
    {
        if (!matchingInliers[i])
            continue;

        const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
        const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];
        cv::Mat p3dC1;

        Triangulate(kp1, kp2, P1, P2, p3dC1);

        if (!std::isfinite(p3dC1.at<float>(0)) || !std::isfinite(p3dC1.at<float>(1)) || !std::isfinite(p3dC1.at<float>(2)))
        {
            vbGood[vMatches12[i].first] = false;
            continue;
        }

        // Check parallax
        cv::Mat normal1 = p3dC1 - O1;
        double dist1 = cv::norm(normal1);

        cv::Mat normal2 = p3dC1 - O2;
        double dist2 = cv::norm(normal2);

        float cosParallax = static_cast<float>(normal1.dot(normal2) / (dist1 * dist2));

        // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        if (p3dC1.at<float>(2) <= 0 && cosParallax < 0.99998)
            continue;

        // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        cv::Mat p3dC2 = R * p3dC1 + t;

        if (p3dC2.at<float>(2) <= 0 && cosParallax < 0.99998)
            continue;

        // Check reprojection error in first image
        float im1x, im1y;
        float invZ1 = 1.0f / p3dC1.at<float>(2);
        im1x = fx * p3dC1.at<float>(0) * invZ1 + cx;
        im1y = fy * p3dC1.at<float>(1) * invZ1 + cy;

        float squareError1 = (im1x - kp1.pt.x) * (im1x - kp1.pt.x) + (im1y - kp1.pt.y) * (im1y - kp1.pt.y);

        if (squareError1 > th2)
            continue;

        // Check reprojection error in second image
        float im2x, im2y;
        float invZ2 = 1.0f / p3dC2.at<float>(2);
        im2x = fx * p3dC2.at<float>(0) * invZ2 + cx;
        im2y = fy * p3dC2.at<float>(1) * invZ2 + cy;

        float squareError2 = (im2x - kp2.pt.x) * (im2x - kp2.pt.x) + (im2y - kp2.pt.y) * (im2y - kp2.pt.y);

        if (squareError2 > th2)
            continue;

        vCosParallax.push_back(cosParallax);
        points3d[vMatches12[i].first] = cv::Point3f(p3dC1.at<float>(0), p3dC1.at<float>(1), p3dC1.at<float>(2));
        nGood++;

        if (cosParallax < 0.99998)
            vbGood[vMatches12[i].first] = true;
    }

    if (nGood > 0)
    {
        sort(vCosParallax.begin(), vCosParallax.end());

        size_t idx = std::min(50, int(vCosParallax.size() - 1));
        parallax = static_cast<float>(acos(vCosParallax[idx]) * 180 / CV_PI);
    }
    else
        parallax = 0;

    return nGood;
}

void OrbInitializer::DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t)
{
    cv::Mat u, w, vt;
    cv::SVD::compute(E, w, u, vt);

    u.col(2).copyTo(t);
    t = t / cv::norm(t);

    cv::Mat W(3, 3, CV_32F, cv::Scalar(0));
    W.at<float>(0, 1) = -1;
    W.at<float>(1, 0) = 1;
    W.at<float>(2, 2) = 1;

    R1 = u * W * vt;
    if (cv::determinant(R1) < 0)
        R1 = -R1;

    R2 = u * W.t() * vt;
    if (cv::determinant(R2) < 0)
        R2 = -R2;
}