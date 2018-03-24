/**
 * Copyright (C) 2017 Chalmers Revere
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 */


#include "sim3solver.hpp"
#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>

//#include "ORBmatcher.h"


Sim3Solver::Sim3Solver(std::shared_ptr<OrbFrame> localKeyFrame1, std::shared_ptr<OrbFrame> localKeyFrame2, const std::vector<std::shared_ptr<OrbMapPoint>> &matchedMapPoints, const bool bFixScale):
  m_keyFrame1()
, m_keyFrame2()
, mvX3Dc1()
, mvX3Dc2()
, m_mapPoints1()
, m_mapPoints2()
, m_matchedMapPoints() 
, mvnIndices1()
, m_SigmaSquare1()
, m_SigmaSquare2()
, m_maxError1()
, m_maxError2()
, Ni()
, m_sizeMatchedMP()
, mR12i()
, mt12i()
, ms12i()
, mT12i()
, mT21i()
, mvbInliersi()
, mnInliersi()
, mnIterations(0)
, mvbBestInliers()
, mnBestInliers(0)
, mBestT12()
, mBestRotation()
, mBestTranslation()
, mBestScale()
, mbFixScale(bFixScale)
, mvAllIndices()
, mvP1im1()
, mvP2im2()
, mRansacProb()
, mRansacMinInliers()
, mRansacMaxIts()
, mTh()
, mSigma2()
, m_calibrationK1()
, m_calibrationK2()
{
	m_keyFrame1 = localKeyFrame1;
    m_keyFrame2 = localKeyFrame2;

    std::vector<std::shared_ptr<OrbMapPoint>> keyFrameMP1 = localKeyFrame1->GetMapPointMatches();

    m_sizeMatchedMP = matchedMapPoints.size();

    m_mapPoints1.reserve(m_sizeMatchedMP);
    m_mapPoints2.reserve(m_sizeMatchedMP);
    m_matchedMapPoints = matchedMapPoints;
    mvnIndices1.reserve(m_sizeMatchedMP);
    mvX3Dc1.reserve(m_sizeMatchedMP);
    mvX3Dc2.reserve(m_sizeMatchedMP);

    cv::Mat Rcw1 = localKeyFrame1->GetRotation();
    cv::Mat tcw1 = localKeyFrame1->GetTranslation();
    cv::Mat Rcw2 = localKeyFrame2->GetRotation();
    cv::Mat tcw2 = localKeyFrame2->GetTranslation();

    mvAllIndices.reserve(m_sizeMatchedMP);

    size_t idx=0;
    for(int i1=0; i1<m_sizeMatchedMP; i1++)
    {
        if(matchedMapPoints[i1])
        {
            std::shared_ptr<OrbMapPoint> mapPoint1 = keyFrameMP1[i1];
            std::shared_ptr<OrbMapPoint> mapPoint2 = matchedMapPoints[i1];

            if(!mapPoint1)
                continue;

            if(mapPoint1->IsCorrupt() || mapPoint2->IsCorrupt())
                continue;

            int indexKF1 = mapPoint1->GetObeservationIndexOfKeyFrame(localKeyFrame1);
            int indexKF2 = mapPoint2->GetObeservationIndexOfKeyFrame(localKeyFrame2);

            if(indexKF1<0 || indexKF2<0)
                continue;

            std::vector<cv::KeyPoint> keyPointVector1 = localKeyFrame1->GetUndistortedKeyPoints();

            std::vector<cv::KeyPoint> keyPointVector2 = localKeyFrame2->GetUndistortedKeyPoints();

            const cv::KeyPoint &kp1 = keyPointVector1[indexKF1]; //localKeyFrame1->mvKeysUn[indexKF1];
            const cv::KeyPoint &kp2 = keyPointVector2[indexKF2]; //localKeyFrame2->mvKeysUn[indexKF2];

            std::vector<float> sigma2Vector1;
            std::vector<float> sigma2Vector2;

            sigma2Vector1 = localKeyFrame1->GetLevelSigma2();
            sigma2Vector2 = localKeyFrame2->GetLevelSigma2();

            const float sigmaSquare1 = sigma2Vector1[kp1.octave]; // localKeyFrame1->GetLevelSigma2[kp1.octave];
            const float sigmaSquare2 = sigma2Vector2[kp2.octave];// localKeyFrame2->GetLevelSigma2[kp2.octave];

            m_maxError1.push_back(static_cast<unsigned long int>(9.210*sigmaSquare1));
            m_maxError2.push_back(static_cast<unsigned long int>(9.210*sigmaSquare2));

            m_mapPoints1.push_back(mapPoint1);
            m_mapPoints2.push_back(mapPoint2);
            mvnIndices1.push_back(i1);

            cv::Mat X3D1w = mapPoint1->GetWorldPosition();
            mvX3Dc1.push_back(Rcw1*X3D1w+tcw1);

            cv::Mat X3D2w = mapPoint2->GetWorldPosition();
            mvX3Dc2.push_back(Rcw2*X3D2w+tcw2);

            mvAllIndices.push_back(idx);
            idx++;
        }
    }

    m_calibrationK1 = localKeyFrame1->GetCalibrationMatrix();
    m_calibrationK2 = localKeyFrame2->GetCalibrationMatrix();

    FromCameraToImage(mvX3Dc1,mvP1im1,m_calibrationK1);
    FromCameraToImage(mvX3Dc2,mvP2im2,m_calibrationK2);

    SetRansacParameters();
}



Sim3Solver::~Sim3Solver()
{
}

void Sim3Solver::SetRansacParameters(double probability, int minInliers, int maxIterations)
{
    mRansacProb = probability;
    mRansacMinInliers = minInliers;
    mRansacMaxIts = maxIterations;    

    Ni = m_mapPoints1.size(); // number of correspondences

    mvbInliersi.resize(Ni);

    // Adjust Parameters according to number of correspondences
    float epsilon = (float)mRansacMinInliers/Ni;

    // Set RANSAC iterations according to probability, epsilon, and max iterations
    int nIterations;

    if(mRansacMinInliers==Ni)
        nIterations=1;
    else
        nIterations = static_cast<int>(std::ceil(log(1-mRansacProb)/log(1-pow(epsilon,3))));

    mRansacMaxIts = std::max(1,std::min(nIterations,mRansacMaxIts));

    mnIterations = 0;
}

cv::Mat Sim3Solver::iterate(int nIterations, bool &bNoMore, std::vector<bool> &vbInliers, int &nInliers)
{
    bNoMore = false;
    vbInliers = std::vector<bool>(m_sizeMatchedMP,false);
    nInliers=0;

    if(Ni<mRansacMinInliers)
    {
        bNoMore = true;
        return cv::Mat();
    }

    std::vector<size_t> vAvailableIndices;

    cv::Mat P3Dc1i(3,3,CV_32F);
    cv::Mat P3Dc2i(3,3,CV_32F);

    int nCurrentIterations = 0;
    while(mnIterations<mRansacMaxIts && nCurrentIterations<nIterations)
    {
        nCurrentIterations++;
        mnIterations++;

        vAvailableIndices = mvAllIndices;

        // Get min set of points
        for(short i = 0; i < 3; ++i)
        {
            int randi = randomInt(0, vAvailableIndices.size()-1);

            int idx = vAvailableIndices[randi];

            mvX3Dc1[idx].copyTo(P3Dc1i.col(i));
            mvX3Dc2[idx].copyTo(P3Dc2i.col(i));

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        ComputeSim3(P3Dc1i,P3Dc2i);

        CheckInliers();

        if(mnInliersi>=mnBestInliers)
        {
            mvbBestInliers = mvbInliersi;
            mnBestInliers = mnInliersi;
            mBestT12 = mT12i.clone();
            mBestRotation = mR12i.clone();
            mBestTranslation = mt12i.clone();
            mBestScale = ms12i;

            if(mnInliersi>mRansacMinInliers)
            {
                nInliers = mnInliersi;
                for(int i=0; i<Ni; i++)
                    if(mvbInliersi[i])
                        vbInliers[mvnIndices1[i]] = true;
                return mBestT12;
            }
        }
    }

    if(mnIterations>=mRansacMaxIts)
        bNoMore=true;

    return cv::Mat();
}

cv::Mat Sim3Solver::find(std::vector<bool> &vbInliers12, int &nInliers)
{
    bool bFlag;
    return iterate(mRansacMaxIts,bFlag,vbInliers12,nInliers);
}

void Sim3Solver::ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C)
{
    cv::reduce(P,C,1,CV_REDUCE_SUM);
    C = C/P.cols;

    for(int i=0; i<P.cols; i++)
    {
        Pr.col(i)=P.col(i)-C;
    }
}

void Sim3Solver::ComputeSim3(cv::Mat &P1, cv::Mat &P2)
{
    // Custom implementation of:
    // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

    // Step 1: Centroid and relative coordinates

    cv::Mat Pr1(P1.size(),P1.type()); // Relative coordinates to centroid (set 1)
    cv::Mat Pr2(P2.size(),P2.type()); // Relative coordinates to centroid (set 2)
    cv::Mat O1(3,1,Pr1.type()); // Centroid of P1
    cv::Mat O2(3,1,Pr2.type()); // Centroid of P2

    ComputeCentroid(P1,Pr1,O1);
    ComputeCentroid(P2,Pr2,O2);

    // Step 2: Compute M matrix

    cv::Mat M = Pr2*Pr1.t();

    // Step 3: Compute N matrix

    double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

    cv::Mat N(4,4,P1.type());

    N11 = M.at<float>(0,0)+M.at<float>(1,1)+M.at<float>(2,2);
    N12 = M.at<float>(1,2)-M.at<float>(2,1);
    N13 = M.at<float>(2,0)-M.at<float>(0,2);
    N14 = M.at<float>(0,1)-M.at<float>(1,0);
    N22 = M.at<float>(0,0)-M.at<float>(1,1)-M.at<float>(2,2);
    N23 = M.at<float>(0,1)+M.at<float>(1,0);
    N24 = M.at<float>(2,0)+M.at<float>(0,2);
    N33 = -M.at<float>(0,0)+M.at<float>(1,1)-M.at<float>(2,2);
    N34 = M.at<float>(1,2)+M.at<float>(2,1);
    N44 = -M.at<float>(0,0)-M.at<float>(1,1)+M.at<float>(2,2);

    N = (cv::Mat_<float>(4,4) << N11, N12, N13, N14,
                                 N12, N22, N23, N24,
                                 N13, N23, N33, N34,
                                 N14, N24, N34, N44);


    // Step 4: Eigenvector of the highest eigenvalue

    cv::Mat eval, evec;

    cv::eigen(N,eval,evec); //evec[0] is the quaternion of the desired rotation

    cv::Mat vec(1,3,evec.type());
    (evec.row(0).colRange(1,4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)

    // Rotation angle. sin is the norm of the imaginary part, cos is the real part
    double ang=atan2(norm(vec),evec.at<float>(0,0));

    vec = 2*ang*vec/norm(vec); //Angle-axis representation. quaternion angle is the half

    mR12i.create(3,3,P1.type());

    cv::Rodrigues(vec,mR12i); // computes the rotation matrix from angle-axis

    // Step 5: Rotate set 2

    cv::Mat P3 = mR12i*Pr2;

    // Step 6: Scale

    if(!mbFixScale)
    {
        double nom = Pr1.dot(P3);
        cv::Mat aux_P3(P3.size(),P3.type());
        aux_P3=P3;
        cv::pow(P3,2,aux_P3);
        double den = 0;

        for(int i=0; i<aux_P3.rows; i++)
        {
            for(int j=0; j<aux_P3.cols; j++)
            {
                den+=aux_P3.at<float>(i,j);
            }
        }

        ms12i = static_cast<float>(nom/den);
    }
    else
        ms12i = 1.0f;

    // Step 7: Translation

    mt12i.create(1,3,P1.type());
    mt12i = O1 - ms12i*mR12i*O2;

    // Step 8: Transformation

    // Step 8.1 T12
    mT12i = cv::Mat::eye(4,4,P1.type());

    cv::Mat sR = ms12i*mR12i;

    sR.copyTo(mT12i.rowRange(0,3).colRange(0,3));
    mt12i.copyTo(mT12i.rowRange(0,3).col(3));

    // Step 8.2 T21

    mT21i = cv::Mat::eye(4,4,P1.type());

    cv::Mat sRinv = (1.0/ms12i)*mR12i.t();

    sRinv.copyTo(mT21i.rowRange(0,3).colRange(0,3));
    cv::Mat tinv = -sRinv*mt12i;
    tinv.copyTo(mT21i.rowRange(0,3).col(3));
}


void Sim3Solver::CheckInliers()
{
    std::vector<cv::Mat> vP1im2, vP2im1;
    Project(mvX3Dc2,vP2im1,mT12i,m_calibrationK1);
    Project(mvX3Dc1,vP1im2,mT21i,m_calibrationK2);

    mnInliersi=0;

    for(size_t i=0; i<mvP1im1.size(); i++)
    {
        cv::Mat dist1 = mvP1im1[i]-vP2im1[i];
        cv::Mat dist2 = vP1im2[i]-mvP2im2[i];

        const float err1 = static_cast<float>(dist1.dot(dist1));
        const float err2 = static_cast<float>(dist2.dot(dist2));

        if(err1<m_maxError1[i] && err2<m_maxError2[i])
        {
            mvbInliersi[i]=true;
            mnInliersi++;
        }
        else
            mvbInliersi[i]=false;
    }
}


cv::Mat Sim3Solver::GetEstimatedRotation()
{
    return mBestRotation.clone();
}

cv::Mat Sim3Solver::GetEstimatedTranslation()
{
    return mBestTranslation.clone();
}

float Sim3Solver::GetEstimatedScale()
{
    return mBestScale;
}

void Sim3Solver::Project(const std::vector<cv::Mat> &vP3Dw, std::vector<cv::Mat> &vP2D, cv::Mat Tcw, cv::Mat K)
{
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    const float &fx = K.at<float>(0,0);
    const float &fy = K.at<float>(1,1);
    const float &cx = K.at<float>(0,2);
    const float &cy = K.at<float>(1,2);

    vP2D.clear();
    vP2D.reserve(vP3Dw.size());

    for(size_t i=0, iend=vP3Dw.size(); i<iend; i++)
    {
        cv::Mat P3Dc = Rcw*vP3Dw[i]+tcw;
        const float invz = 1/(P3Dc.at<float>(2));
        const float x = P3Dc.at<float>(0)*invz;
        const float y = P3Dc.at<float>(1)*invz;

        vP2D.push_back((cv::Mat_<float>(2,1) << fx*x+cx, fy*y+cy));
    }
}

void Sim3Solver::FromCameraToImage(const std::vector<cv::Mat> &vP3Dc, std::vector<cv::Mat> &vP2D, cv::Mat K)
{
    const float &fx = K.at<float>(0,0);
    const float &fy = K.at<float>(1,1);
    const float &cx = K.at<float>(0,2);
    const float &cy = K.at<float>(1,2);

    vP2D.clear();
    vP2D.reserve(vP3Dc.size());

    for(size_t i=0, iend=vP3Dc.size(); i<iend; i++)
    {
        const float invz = 1/(vP3Dc[i].at<float>(2));
        const float x = vP3Dc[i].at<float>(0)*invz;
        const float y = vP3Dc[i].at<float>(1)*invz;

        vP2D.push_back((cv::Mat_<float>(2,1) << fx*x+cx, fy*y+cy));
    }
}

int Sim3Solver::randomInt(int max, int min){

	int d = max - min + 1;
	return int(((double)rand()/((double)RAND_MAX + 1.0)) * d) + min;
}


