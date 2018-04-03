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

#include <orbkeyframe.hpp>

long unsigned int OrbKeyFrame::nNextId=0;

OrbKeyFrame::OrbKeyFrame(std::shared_ptr<OrbFrame> F, std::shared_ptr<OrbMap> pMap, std::shared_ptr<OrbKeyFrameDatabase> pKFDB):
        mnFrameId(F->mnId), mTimeStamp(F->mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
        mfGridElementWidthInv(F->mfGridElementWidthInv), mfGridElementHeightInv(F->mfGridElementHeightInv),
        mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
        mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
        fx(F->fx), fy(F->fy), cx(F->cx), cy(F->cy), invfx(F->invfx), invfy(F->invfy),
        mbf(F->mbf), mb(F->mb), mThDepth(F->mThDepth), N(F->N), mvKeys(F->mvKeys), mvKeysUn(F->mvKeysUn),
        mvuRight(F->mvuRight), mvDepth(F->mvDepth), mDescriptors(F->mDescriptors.clone()),
        mBowVec(F->mBowVec), mFeatVec(F->mFeatVec), mnScaleLevels(F->mnScaleLevels), mfScaleFactor(F->mfScaleFactor),
        mfLogScaleFactor(F->mfLogScaleFactor), mvScaleFactors(F->mvScaleFactors), mvLevelSigma2(F->mvLevelSigma2),
        mvInvLevelSigma2(F->mvInvLevelSigma2), mnMinX(static_cast<const int>(F->mnMinX)), mnMinY(
        static_cast<const int>(F->mnMinY)), mnMaxX(static_cast<const int>(F->mnMaxX)),
        mnMaxY(static_cast<const int>(F->mnMaxY)), mK(F->mK), mvpMapPoints(F->mvpMapPoints), mpKeyFrameDB(pKFDB),
        mpORBvocabulary(F->mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
        mbToBeErased(false), mbBad(false), mHalfBaseline(F->mb/2), mpMap(pMap)
{
    mnId=nNextId++;

    mGrid.resize((unsigned long) mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize((unsigned long) mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F->mGrid[i][j];
    }

    SetPose(F->mTcw);
}

void OrbKeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        std::vector<cv::Mat> vCurrentDesc = Orbconverter::toDescriptorVector(mDescriptors);
        // Feature std::vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform4(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void OrbKeyFrame::SetPose(const cv::Mat &Tcw_)
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    Tcw_.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    Ow = -Rwc*tcw;

    Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Ow.copyTo(Twc.rowRange(0,3).col(3));
    cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
    Cw = Twc*center;
}

cv::Mat OrbKeyFrame::GetPose()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return Tcw.clone();
}

cv::Mat OrbKeyFrame::GetPoseInverse()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return Twc.clone();
}

cv::Mat OrbKeyFrame::GetCameraCenter()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return Ow.clone();
}

cv::Mat OrbKeyFrame::GetStereoCenter()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return Cw.clone();
}


cv::Mat OrbKeyFrame::GetRotation()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).colRange(0,3).clone();
}

cv::Mat OrbKeyFrame::GetTranslation()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).col(3).clone();
}

void OrbKeyFrame::AddConnection(std::shared_ptr<OrbKeyFrame> pKF, const int &weight)
{
    {
        std::unique_lock<std::mutex> lock(mMutexConnections);
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void OrbKeyFrame::UpdateBestCovisibles()
{
    std::unique_lock<std::mutex> lock(mMutexConnections);
    std::vector<std::pair<int,std::shared_ptr<OrbKeyFrame>> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(std::map<std::shared_ptr<OrbKeyFrame>,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        vPairs.push_back(make_pair(mit->second,mit->first));

    std::sort(vPairs.begin(),vPairs.end());
    std::list<std::shared_ptr<OrbKeyFrame>> lKFs;
    std::list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    mvpOrderedConnectedKeyFrames = std::vector<std::shared_ptr<OrbKeyFrame>>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = std::vector<int>(lWs.begin(), lWs.end());
}

std::set<std::shared_ptr<OrbKeyFrame>> OrbKeyFrame::GetConnectedKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexConnections);
    std::set<std::shared_ptr<OrbKeyFrame>> s;
    for(std::map<std::shared_ptr<OrbKeyFrame>,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

std::vector<std::shared_ptr<OrbKeyFrame>> OrbKeyFrame::GetVectorCovisibleKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

std::vector<std::shared_ptr<OrbKeyFrame>> OrbKeyFrame::GetBestCovisibilityKeyFrames(const int &n)
{
    std::unique_lock<std::mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<n)
        return mvpOrderedConnectedKeyFrames;
    else
        return std::vector<std::shared_ptr<OrbKeyFrame>>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

std::vector<std::shared_ptr<OrbKeyFrame>> OrbKeyFrame::GetCovisiblesByWeight(const int &w)
{
    std::unique_lock<std::mutex> lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
        return std::vector<std::shared_ptr<OrbKeyFrame>>();

    std::vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,OrbKeyFrame::weightComp);
    if(it==mvOrderedWeights.end())
        return std::vector<std::shared_ptr<OrbKeyFrame>>();
    else
    {
        int n = static_cast<int>(it - mvOrderedWeights.begin());
        return std::vector<std::shared_ptr<OrbKeyFrame>>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

int OrbKeyFrame::GetWeight(std::shared_ptr<OrbKeyFrame> pKF)
{
    std::unique_lock<std::mutex> lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

void OrbKeyFrame::AddMapPoint(std::shared_ptr<OrbMapPoint> pMP, const size_t &idx)
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=pMP;
}

void OrbKeyFrame::EraseMapPointMatch(const size_t &idx)
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=static_cast<std::shared_ptr<OrbMapPoint>>(NULL);
}

void OrbKeyFrame::EraseMapPointMatch(std::shared_ptr<OrbMapPoint> pMP)
{
    int idx = pMP->GetObeservationIndexOfKeyFrame(std::shared_ptr<OrbKeyFrame>(this));
    if(idx>=0)
        mvpMapPoints[idx]=static_cast<std::shared_ptr<OrbMapPoint>>(NULL);
}


void OrbKeyFrame::ReplaceMapPointMatch(const size_t &idx, std::shared_ptr<OrbMapPoint> pMP)
{
    mvpMapPoints[idx]=pMP;
}

std::set<std::shared_ptr<OrbMapPoint>> OrbKeyFrame::GetMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    std::set<std::shared_ptr<OrbMapPoint>> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        std::shared_ptr<OrbMapPoint> pMP = mvpMapPoints[i];
        if(!pMP->IsCorrupt())
            s.insert(pMP);
    }
    return s;
}

int OrbKeyFrame::TrackedMapPoints(const int &minObs)
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        std::shared_ptr<OrbMapPoint> pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->IsCorrupt())
            {
                if(bCheckObs)
                {
                    if(mvpMapPoints[i]->GetObservingKeyFrameCount()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

std::vector<std::shared_ptr<OrbMapPoint>> OrbKeyFrame::GetMapPointMatches()
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

std::shared_ptr<OrbMapPoint> OrbKeyFrame::GetMapPoint(const size_t &idx)
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

void OrbKeyFrame::UpdateConnections()
{
    std::map<std::shared_ptr<OrbKeyFrame>,int> KFcounter;

    std::vector<std::shared_ptr<OrbMapPoint>> vpMP;

    {
        std::unique_lock<std::mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(std::vector<std::shared_ptr<OrbMapPoint>>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        std::shared_ptr<OrbMapPoint> pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->IsCorrupt())
            continue;

        std::map<std::shared_ptr<OrbKeyFrame>,size_t> observations = pMP->GetObservingKeyframes();

        for(std::map<std::shared_ptr<OrbKeyFrame>,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId)
                continue;
            KFcounter[mit->first]++;
        }
    }

    // This should not happen
    if(KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    std::shared_ptr<OrbKeyFrame> pKFmax=NULL;
    int th = 15;

    std::vector<std::pair<int,std::shared_ptr<OrbKeyFrame>> > vPairs;
    vPairs.reserve(KFcounter.size());
    for(std::map<std::shared_ptr<OrbKeyFrame>,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection(std::shared_ptr<OrbKeyFrame>(this), mit->second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(std::shared_ptr<OrbKeyFrame>(this),nmax);
    }

    std::sort(vPairs.begin(),vPairs.end());
    std::list<std::shared_ptr<OrbKeyFrame>> lKFs;
    std::list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        std::unique_lock<std::mutex> lockCon(mMutexConnections);

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = std::vector<std::shared_ptr<OrbKeyFrame>>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = std::vector<int>(lWs.begin(), lWs.end());

        if(mbFirstConnection && mnId!=0)
        {
            mpParent = mvpOrderedConnectedKeyFrames.front();
            mpParent->AddChild(std::shared_ptr<OrbKeyFrame>(this));
            mbFirstConnection = false;
        }

    }
}

void OrbKeyFrame::AddChild(std::shared_ptr<OrbKeyFrame> pKF)
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void OrbKeyFrame::EraseChild(std::shared_ptr<OrbKeyFrame> pKF)
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

void OrbKeyFrame::ChangeParent(std::shared_ptr<OrbKeyFrame> pKF)
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    mpParent = pKF;
    pKF->AddChild(std::shared_ptr<OrbKeyFrame>(this));
}

std::set<std::shared_ptr<OrbKeyFrame>> OrbKeyFrame::GetChilds()
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

std::shared_ptr<OrbKeyFrame> OrbKeyFrame::GetParent()
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool OrbKeyFrame::hasChild(std::shared_ptr<OrbKeyFrame> pKF)
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    return static_cast<bool>(mspChildrens.count(pKF));
}

void OrbKeyFrame::AddLoopEdge(std::shared_ptr<OrbKeyFrame> pKF)
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

std::set<std::shared_ptr<OrbKeyFrame>> OrbKeyFrame::GetLoopEdges()
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void OrbKeyFrame::SetNotErase()
{
    std::unique_lock<std::mutex> lock(mMutexConnections);
    mbNotErase = true;
}

void OrbKeyFrame::SetErase()
{
    {
        std::unique_lock<std::mutex> lock(mMutexConnections);
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();
    }
}

void OrbKeyFrame::SetBadFlag()
{
    {
        std::unique_lock<std::mutex> lock(mMutexConnections);
        if(mnId==0)
            return;
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }

    for(std::map<std::shared_ptr<OrbKeyFrame>,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        mit->first->EraseConnection(std::shared_ptr<OrbKeyFrame>(this));

    for(size_t i=0; i<mvpMapPoints.size(); i++)
        if(mvpMapPoints[i])
            mvpMapPoints[i]->EraseObservingKeyframe(std::shared_ptr<OrbKeyFrame>(this));
    {
        std::unique_lock<std::mutex> lock(mMutexConnections);
        std::unique_lock<std::mutex> lock1(mMutexFeatures);

        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree
        std::set<std::shared_ptr<OrbKeyFrame>> sParentCandidates;
        sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            std::shared_ptr<OrbKeyFrame> pC;
            std::shared_ptr<OrbKeyFrame> pP;

            for(std::set<std::shared_ptr<OrbKeyFrame>>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                std::shared_ptr<OrbKeyFrame> pKF = *sit;
                if(pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                std::vector<std::shared_ptr<OrbKeyFrame>> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(std::set<std::shared_ptr<OrbKeyFrame>>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            int w = pKF->GetWeight(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if(bContinue)
            {
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!mspChildrens.empty())
            for(std::set<std::shared_ptr<OrbKeyFrame>>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);
            }

        mpParent->EraseChild(std::shared_ptr<OrbKeyFrame>(this));
        mTcp = Tcw*mpParent->GetPoseInverse();
        mbBad = true;
    }


    mpMap->DeleteOrbKeyFrame(std::shared_ptr<OrbKeyFrame>(this));
    mpKeyFrameDB->erase(std::shared_ptr<OrbKeyFrame>(this));
}

bool OrbKeyFrame::isBad()
{
    std::unique_lock<std::mutex> lock(mMutexConnections);
    return mbBad;
}

void OrbKeyFrame::EraseConnection(std::shared_ptr<OrbKeyFrame> pKF)
{
    bool bUpdate = false;
    {
        std::unique_lock<std::mutex> lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}

std::vector<size_t> OrbKeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    std::vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = std::max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = std::min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = std::max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = std::min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const std::vector<size_t> vCell = mGrid[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool OrbKeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

cv::Mat OrbKeyFrame::UnprojectStereo(int i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        std::unique_lock<std::mutex> lock(mMutexPose);
        return Twc.rowRange(0,3).colRange(0,3)*x3Dc+Twc.rowRange(0,3).col(3);
    }
    else
        return cv::Mat();
}

float OrbKeyFrame::ComputeSceneMedianDepth(const int q)
{
    std::vector<std::shared_ptr<OrbMapPoint>> vpMapPoints;
    cv::Mat Tcw_;
    {
        std::unique_lock<std::mutex> lock(mMutexFeatures);
        std::unique_lock<std::mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        Tcw_ = Tcw.clone();
    }

    std::vector<float> vDepths;
    vDepths.reserve(N);
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    for(int i=0; i<N; i++)
    {
        if(mvpMapPoints[i])
        {
            std::shared_ptr<OrbMapPoint> pMP = mvpMapPoints[i];
            cv::Mat x3Dw = pMP->GetWorldPosition();
            float z = static_cast<float>(Rcw2.dot(x3Dw) + zcw);
            vDepths.push_back(z);
        }
    }

    std::sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}