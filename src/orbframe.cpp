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
#include <orbframe.hpp>
#include <orbmatcher.hpp>

long unsigned int OrbFrame::nNextId=0;
bool OrbFrame::mbInitialComputations=true;
float OrbFrame::cx, OrbFrame::cy, OrbFrame::fx, OrbFrame::fy, OrbFrame::invfx, OrbFrame::invfy;
float OrbFrame::mnMinX, OrbFrame::mnMinY, OrbFrame::mnMaxX, OrbFrame::mnMaxY;
float OrbFrame::mfGridElementWidthInv, OrbFrame::mfGridElementHeightInv;

//Copy Constructor
OrbFrame::OrbFrame(const std::shared_ptr<OrbFrame>&frame)
        :mpORBvocabulary(frame->mpORBvocabulary), mpORBextractorLeft(frame->mpORBextractorLeft), mpORBextractorRight(frame->mpORBextractorRight),
         mTimeStamp(frame->mTimeStamp), mK(frame->mK.clone()), mDistCoef(frame->mDistCoef.clone()),
         mbf(frame->mbf), mb(frame->mb), mThDepth(frame->mThDepth), N(frame->N), mvKeys(frame->mvKeys),
         mvKeysRight(frame->mvKeysRight), mvKeysUn(frame->mvKeysUn),  mvuRight(frame->mvuRight),
         mvDepth(frame->mvDepth), mBowVec(frame->mBowVec), mFeatVec(frame->mFeatVec),
         mDescriptors(frame->mDescriptors.clone()), mDescriptorsRight(frame->mDescriptorsRight.clone()),
         mvpMapPoints(frame->mvpMapPoints), mvbOutlier(frame->mvbOutlier), mnId(frame->mnId),
         mpReferenceKF(frame->mpReferenceKF), mnScaleLevels(frame->mnScaleLevels),
         mfScaleFactor(frame->mfScaleFactor), mfLogScaleFactor(frame->mfLogScaleFactor),
         mvScaleFactors(frame->mvScaleFactors), mvInvScaleFactors(frame->mvInvScaleFactors),
         mvLevelSigma2(frame->mvLevelSigma2), mvInvLevelSigma2(frame->mvInvLevelSigma2)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
    {
        for(int j=0; j<FRAME_GRID_ROWS; j++)
        {
            mGrid[i][j]=frame->mGrid[i][j];
        }
    }

    if(!frame->mTcw.empty())
    {
        SetPose(frame->mTcw);
    }
}

OrbFrame::OrbFrame(const cv::Mat &leftImage, const cv::Mat &rightImage, const double &timeStamp,
                   std::shared_ptr<OrbExtractor> leftExtractor, std::shared_ptr<OrbExtractor> rightExtractor,
                   std::shared_ptr<OrbVocabulary> orbVocabulary, cv::Mat &calibrationMatrix, cv::Mat &distanceCoefficient, const float &stereoBaseline, const float &depthThreshold)
        :mpORBvocabulary(orbVocabulary),mpORBextractorLeft(leftExtractor),mpORBextractorRight(rightExtractor), mTimeStamp(timeStamp),
         mK(calibrationMatrix.clone()),mDistCoef(distanceCoefficient.clone()), mbf(stereoBaseline), mThDepth(depthThreshold), mpReferenceKF(static_cast<std::shared_ptr<OrbKeyFrame>>(NULL))
{
    // Frame ID
    mnId = nNextId++;
    CommonSetup();

    // ORB extraction
    std::thread threadLeft(&OrbFrame::ExtractORB, this, 0, leftImage);
    std::thread threadRight(&OrbFrame::ExtractORB, this, 1, rightImage);
    threadLeft.join();
    threadRight.join();

    ComputeStereoMatches();

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        InitialComputation(calibrationMatrix, leftImage);
    }

    mb = mbf/fx;
    AssignFeaturesToGrid();
}

OrbFrame::OrbFrame(const cv::Mat &greyImage, const cv::Mat &imageDepth, const double &timeStamp,
                   std::shared_ptr<OrbExtractor> extractor, std::shared_ptr<OrbVocabulary> orbVocabulary,
                   cv::Mat &calibrationMatrix, cv::Mat &distanceCoefficient, const float &stereoBaseline,
                   const float &depthThreshold)
        :mpORBvocabulary(orbVocabulary), mpORBextractorLeft(extractor),
         mpORBextractorRight(static_cast<std::shared_ptr<OrbExtractor>>(NULL)), mTimeStamp(timeStamp),
         mK(calibrationMatrix.clone()),mDistCoef(distanceCoefficient.clone()), mbf(stereoBaseline), mThDepth(depthThreshold)
{
    // Frame ID
    mnId = nNextId++;
    CommonSetup();

    // ORB extraction
    ExtractORB(0, greyImage);

    ComputeStereoFromRGBD(imageDepth);



    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        InitialComputation(calibrationMatrix, greyImage);
    }

    mb = mbf/fx;
    AssignFeaturesToGrid();
}


OrbFrame::OrbFrame(const cv::Mat &greyImage, const double &timeStamp, std::shared_ptr<OrbExtractor> extractor,
                   std::shared_ptr<OrbVocabulary> orbVocabulary, cv::Mat &calibrationMatrix, cv::Mat &distanceCoefficient,
                   const float &stereoBaseline, const float &depthThreshold)
        :mpORBvocabulary(orbVocabulary), mpORBextractorLeft(extractor),
         mpORBextractorRight(static_cast<std::shared_ptr<OrbExtractor>>(NULL)), mTimeStamp(timeStamp),
         mK(calibrationMatrix.clone()), mDistCoef(distanceCoefficient.clone()), mbf(stereoBaseline), mThDepth(depthThreshold)
{
    // Frame ID
    mnId = nNextId++;
    CommonSetup();

    // ORB extraction
    ExtractORB(0, greyImage);

    // Set no stereo information
    mvuRight = std::vector<float>(N,-1);
    mvDepth = std::vector<float>(N,-1);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        InitialComputation(calibrationMatrix, greyImage);
    }

    mb = mbf/fx;
    AssignFeaturesToGrid();
}

void OrbFrame::CommonSetup()
{
    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->getLevels();
    mfScaleFactor = static_cast<float>(mpORBextractorLeft->getScaleFactor());
    mfLogScaleFactor = static_cast<float>(log(mfScaleFactor));
    mvScaleFactors = mpORBextractorLeft->getScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->getInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->getScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->getInverseScaleSigmaSquares();

    N = static_cast<int>(mvKeys.size());

    if(mvKeys.empty())
    {
        return;
    }

    UndistortKeyPoints();

    mvpMapPoints = std::vector<std::shared_ptr<OrbMapPoint>>(static_cast<unsigned long>(N),
                                                             static_cast<std::shared_ptr<OrbMapPoint>>(NULL));
    mvbOutlier = std::vector<bool>(static_cast<unsigned long>(N), false);
}

void OrbFrame::InitialComputation(cv::Mat calibrationMatrix, cv::Mat image)
{
    ComputeImageBounds(image);

    mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
    mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);

    fx = calibrationMatrix.at<float>(0,0);
    fy = calibrationMatrix.at<float>(1,1);
    cx = calibrationMatrix.at<float>(0,2);
    cy = calibrationMatrix.at<float>(1,2);
    invfx = 1.0f/fx;
    invfy = 1.0f/fy;

    mbInitialComputations=false;
}

void OrbFrame::AssignFeaturesToGrid()
{
    int nReserve = static_cast<int>(0.5f * N / (FRAME_GRID_COLS * FRAME_GRID_ROWS));
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
    {
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
        {
            mGrid[i][j].reserve(static_cast<unsigned long>(nReserve));
        }
    }

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &keyPoint = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(keyPoint, nGridPosX, nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(static_cast<unsigned long &&>(i));
    }
}

void OrbFrame::ExtractORB(int rightImage, const cv::Mat &image)
{
    if(rightImage==0)
    {
        mpORBextractorLeft->ExtractFeatures(image, mvKeys, mDescriptors);
    }
    else
    {
        mpORBextractorRight->ExtractFeatures(image, mvKeys, mDescriptors);
    }
}

void OrbFrame::SetPose(cv::Mat cameraPose)
{
    mTcw = cameraPose.clone();
    UpdatePoseMatrices();
}

void OrbFrame::UpdatePoseMatrices()
{
    m_rotation = mTcw.rowRange(0,3).colRange(0,3);
    m_reverseRotation = m_rotation.t();
    m_reversePose = mTcw.rowRange(0,3).col(3);
    m_cameraCenter = -m_rotation.t()*m_reversePose;
}

bool OrbFrame::isInFrustum(std::shared_ptr<OrbMapPoint> mapPoint, float viewingCosLimit)
{
    mapPoint->SetTrackInView(static_cast<unsigned long>(false));

    // 3D in absolute coordinates
    cv::Mat worldPosition = mapPoint->GetWorldPosition();

    // 3D in camera coordinates
    const cv::Mat Pc = m_rotation * worldPosition + m_reversePose;
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
    {
        return false;
    }

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
    {
        return false;
    }
    if(v<mnMinY || v>mnMaxY)
    {
        return false;
    }

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = mapPoint->GetMaxDistanceInvariance();
    const float minDistance = mapPoint->GetMinDistanceInvariance();
    const cv::Mat PO = worldPosition-m_cameraCenter;
    const float dist = static_cast<const float>(cv::norm(PO));

    if(dist<minDistance || dist>maxDistance)
    {
        return false;
    }

    // Check viewing angle
    cv::Mat Pn = mapPoint->GetMeanViewingDirection();

    const float viewCos = static_cast<const float>(PO.dot(Pn) / dist);

    if(viewCos<viewingCosLimit)
    {
        return false;
    }

    // Predict scale in the image
    const int nPredictedLevel = mapPoint->PredictScale(dist, std::shared_ptr<OrbFrame>(this));

    // Data used by the tracking
    mapPoint->SetTrackInView(static_cast<unsigned long>(true));
    mapPoint->SetTrackProjX(u);
    mapPoint->SetTrackProjXR(u - mbf*invz);
    mapPoint->SetTrackProjY(v);
    mapPoint->SetnTrackScaleLevel(nPredictedLevel);
    mapPoint->SetTrackViewCos(viewCos);

    return true;
}


std::vector<size_t> OrbFrame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    std::vector<size_t> indices;
    indices.reserve(static_cast<unsigned long>(N));

    const int nMinCellX = std::max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
    {
        return indices;
    }

    const int nMaxCellX = std::min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
    {
        return indices;
    }

    const int nMinCellY = std::max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
    {
        return indices;
    }

    const int nMaxCellY = std::min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
    {
        return indices;
    }

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const std::vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
            {
                continue;
            }

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                    {
                        continue;
                    }
                    if(maxLevel>=0)
                    {
                        if(kpUn.octave>maxLevel)
                        {
                            continue;
                        }
                    }
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                {
                    indices.push_back(vCell[j]);
                }
            }
        }
    }

    return indices;
}

bool OrbFrame::PosInGrid(const cv::KeyPoint &keyPoint, int &xPosition, int &yPosition)
{
    xPosition = static_cast<int>(round((keyPoint.pt.x - mnMinX) * mfGridElementWidthInv));
    yPosition = static_cast<int>(round((keyPoint.pt.y - mnMinY) * mfGridElementHeightInv));

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(xPosition<0 || xPosition>=FRAME_GRID_COLS || yPosition<0 || yPosition>=FRAME_GRID_ROWS)
    {
        return false;
    }

    return true;
}

void OrbFrame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        std::vector<cv::Mat> currentDescriptors = Orbconverter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform4(currentDescriptors, mBowVec, mFeatVec, 4);
    }
}

void OrbFrame::UndistortKeyPoints()
{
    if(std::abs(mDistCoef.at<float>(0))<0.0001)
    {
        mvKeysUn = mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(static_cast<unsigned long>(N));
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}


void OrbFrame::ComputeImageBounds(const cv::Mat &leftImage)
{
    if(std::abs(mDistCoef.at<float>(0))>0.0001)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=leftImage.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=leftImage.rows;
        mat.at<float>(3,0)=leftImage.cols; mat.at<float>(3,1)=leftImage.rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = std::min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = std::max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = std::min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = std::max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = leftImage.cols;
        mnMinY = 0.0f;
        mnMaxY = leftImage.rows;
    }
}

void OrbFrame::ComputeStereoMatches()
{
    mvuRight = std::vector<float>(static_cast<unsigned long>(N), -1.0f);
    mvDepth = std::vector<float>(static_cast<unsigned long>(N), -1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH + ORBmatcher::TH_LOW)/2;

    const int nRows = mpORBextractorLeft->m_vImagePyramid[0].rows;

    //Assign keypoints to row table
    std::vector<std::vector<size_t> > vRowIndices(nRows, std::vector<size_t>());

    for(int i=0; i<nRows; i++)
    {
        vRowIndices[i].reserve(200);
    }

    const int Nr = static_cast<const int>(mvKeysRight.size());

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = static_cast<const int>(ceil(kpY + r));
        const int minr = static_cast<const int>(floor(kpY - r));

        for(int yi=minr;yi<=maxr;yi++)
        {
            vRowIndices[yi].push_back(static_cast<unsigned long &&>(iR));
        }
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf/minZ;

    // For each left keypoint search a match in the right image
    std::vector<std::pair<int, int> > vDistIdx;
    vDistIdx.reserve(static_cast<unsigned long>(N));

    for(int iL=0; iL < N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const std::vector<size_t> &vCandidates = vRowIndices[(int)vL];

        if(vCandidates.empty())
        {
            continue;
        }

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
        {
            continue;
        }

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
            {
                continue;
            }

            const float &uR = kpR.pt.x;

            if(uR >= minU && uR <= maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row((int) iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist < bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist < thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = static_cast<const float>(round(kpL.pt.x * scaleFactor));
            const float scaledvL = static_cast<const float>(round(kpL.pt.y * scaleFactor));
            const float scaleduR0 = static_cast<const float>(round(uR0 * scaleFactor));

            // sliding window search
            const int w = 5;
            cv::Mat IL = mpORBextractorLeft->m_vImagePyramid[kpL.octave].rowRange(static_cast<int>(scaledvL - w),
                                                                                  static_cast<int>(scaledvL + w + 1)).colRange(
                    static_cast<int>(scaleduL - w), static_cast<int>(scaleduL + w + 1));
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            std::vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->m_vImagePyramid[kpL.octave].cols)
            {
                continue;
            }

            for(int incR=-L; incR<=+L; incR++)
            {
                cv::Mat IR = mpORBextractorRight->m_vImagePyramid[kpL.octave].rowRange(static_cast<int>(scaledvL - w),
                                                                                       static_cast<int>(scaledvL + w + 1)).colRange(
                        static_cast<int>(scaleduR0 + incR - w), static_cast<int>(scaleduR0 + incR + w + 1));
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                float dist = static_cast<float>(cv::norm(IL, IR, cv::NORM_L1));
                if(dist<bestDist)
                {
                    bestDist = static_cast<int>(dist);
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
            {
                continue;
            }

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
            {
                continue;
            }

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave]*(scaleduR0 + (float)bestincR + deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01f;
                    bestuR = static_cast<float>(uL - 0.01);
                }
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(std::pair<int,int>(bestDist,iL));
            }
        }
    }

    sort(vDistIdx.begin(), vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i= static_cast<int>(vDistIdx.size() - 1); i >= 0; i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}

void OrbFrame::ComputeStereoFromRGBD(const cv::Mat &imageDepth)
{
    mvuRight = std::vector<float>(N,-1);
    mvDepth = std::vector<float>(N,-1);

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imageDepth.at<float>((int) v, (int) u);

        if(d>0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;
        }
    }
}

cv::Mat OrbFrame::UnprojectStereo(const int &i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return m_reverseRotation*x3Dc+m_cameraCenter;
    }
    else
        return cv::Mat();
}
