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

OrbKeyFrame::OrbKeyFrame(std::shared_ptr<OrbFrame> frame, std::shared_ptr<OrbMap> map,
                         std::shared_ptr<OrbKeyFrameDatabase> keyFrameDatabase):
        mnFrameId(frame->mnId), mTimeStamp(frame->mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
        mfGridElementWidthInv(frame->mfGridElementWidthInv), mfGridElementHeightInv(frame->mfGridElementHeightInv),
        mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
        mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
        fx(frame->fx), fy(frame->fy), cx(frame->cx), cy(frame->cy), invfx(frame->invfx), invfy(frame->invfy),
        mbf(frame->mbf), mb(frame->mb), mThDepth(frame->mThDepth), N(frame->N), mvKeys(frame->mvKeys), mvKeysUn(frame->mvKeysUn),
        mvuRight(frame->mvuRight), mvDepth(frame->mvDepth), mDescriptors(frame->mDescriptors.clone()),
        mBowVec(frame->mBowVec), mFeatVec(frame->mFeatVec), mnScaleLevels(frame->mnScaleLevels), mfScaleFactor(frame->mfScaleFactor),
        mfLogScaleFactor(frame->mfLogScaleFactor), mvScaleFactors(frame->mvScaleFactors), mvLevelSigma2(frame->mvLevelSigma2),
        mvInvLevelSigma2(frame->mvInvLevelSigma2), mnMinX(static_cast<const int>(frame->mnMinX)), mnMinY(
        static_cast<const int>(frame->mnMinY)), mnMaxX(static_cast<const int>(frame->mnMaxX)),
        mnMaxY(static_cast<const int>(frame->mnMaxY)), mK(frame->mK), m_mapPoints(frame->mvpMapPoints), m_keyFrameDatabase(keyFrameDatabase),
        m_orbVocabulary(frame->mpORBvocabulary), m_isFirstConnection(true), m_parent(NULL), m_shoulNotBeErased(false),
        m_shouldBeErased(false), m_isBad(false), mHalfBaseline(frame->mb/2), m_map(map)
{
    mnId = nNextId++;

    m_grid.resize((unsigned long) mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        m_grid[i].resize((unsigned long) mnGridRows);
        for(int j=0; j<mnGridRows; j++)
        {
            m_grid[i][j] = frame->mGrid[i][j];
        }
    }

    SetPose(frame->mTcw);
}

void OrbKeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        std::vector<cv::Mat> vCurrentDesc = Orbconverter::toDescriptorVector(mDescriptors);
        // Feature std::vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        m_orbVocabulary->transform4(vCurrentDesc, mBowVec, mFeatVec, 4);
    }
}

void OrbKeyFrame::SetPose(const cv::Mat &cameraPose)
{
    std::cout << "cameraPose: " << cameraPose << std::endl;
    std::unique_lock<std::mutex> lock(m_poseMutex);
    cameraPose.copyTo(m_cameraPose);
    cv::Mat Rcw = m_cameraPose.rowRange(0, 3).colRange(0, 3);
    cv::Mat tcw = m_cameraPose.rowRange(0, 3).col(3);
    cv::Mat Rwc = Rcw.t();
    m_cameraCenter = -Rwc * tcw;

    m_reverseCameraPose = cv::Mat::eye(4, 4, m_cameraPose.type());
    Rwc.copyTo(m_reverseCameraPose.rowRange(0, 3).colRange(0, 3));
    m_cameraCenter.copyTo(m_reverseCameraPose.rowRange(0,3).col(3));
    cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
    Cw = m_reverseCameraPose*center;
}

cv::Mat OrbKeyFrame::GetPose()
{
    std::unique_lock<std::mutex> lock(m_poseMutex);
    return m_cameraPose.clone();
}

cv::Mat OrbKeyFrame::GetPoseInverse()
{
    std::unique_lock<std::mutex> lock(m_poseMutex);
    return m_reverseCameraPose.clone();
}

cv::Mat OrbKeyFrame::GetCameraCenter()
{
    std::unique_lock<std::mutex> lock(m_poseMutex);
    return m_cameraCenter.clone();
}

cv::Mat OrbKeyFrame::GetStereoCenter()
{
    std::unique_lock<std::mutex> lock(m_poseMutex);
    return Cw.clone();
}


cv::Mat OrbKeyFrame::GetRotation()
{
    std::unique_lock<std::mutex> lock(m_poseMutex);
    return m_cameraPose.rowRange(0,3).colRange(0,3).clone();
}

cv::Mat OrbKeyFrame::GetTranslation()
{
    std::unique_lock<std::mutex> lock(m_poseMutex);
    return m_cameraPose.rowRange(0,3).col(3).clone();
}

void OrbKeyFrame::AddConnection(std::shared_ptr<OrbKeyFrame> keyFrame, const int &weight)
{
    {
        std::unique_lock<std::mutex> lock(m_connectionsMutex);
        if(!m_connectedKeyFrameWeights.count(keyFrame))
        {
            m_connectedKeyFrameWeights[keyFrame]=weight;
        }
        else if(m_connectedKeyFrameWeights[keyFrame]!=weight)
        {
            m_connectedKeyFrameWeights[keyFrame]=weight;
        }
        else
        {
            return;
        }
    }

    UpdateBestCovisibles();
}

void OrbKeyFrame::UpdateBestCovisibles()
{
    std::unique_lock<std::mutex> lock(m_connectionsMutex);
    std::vector<std::pair<int,std::shared_ptr<OrbKeyFrame>> > vPairs;
    vPairs.reserve(m_connectedKeyFrameWeights.size());
    for(std::map<std::shared_ptr<OrbKeyFrame>,int>::iterator mit=m_connectedKeyFrameWeights.begin(), mend=m_connectedKeyFrameWeights.end(); mit!=mend; mit++)
    {
        vPairs.push_back(make_pair(mit->second,mit->first));
    }

    std::sort(vPairs.begin(),vPairs.end());
    std::list<std::shared_ptr<OrbKeyFrame>> lKFs;
    std::list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    m_orderedConnectedKeyFrames = std::vector<std::shared_ptr<OrbKeyFrame>>(lKFs.begin(),lKFs.end());
    m_orderedWeights = std::vector<int>(lWs.begin(), lWs.end());
}

std::set<std::shared_ptr<OrbKeyFrame>> OrbKeyFrame::GetConnectedKeyFrames()
{
    std::unique_lock<std::mutex> lock(m_connectionsMutex);
    std::set<std::shared_ptr<OrbKeyFrame>> s;
    for(std::map<std::shared_ptr<OrbKeyFrame>,int>::iterator mit=m_connectedKeyFrameWeights.begin();mit!=m_connectedKeyFrameWeights.end();mit++)
    {
        s.insert(mit->first);
    }
    return s;
}

std::vector<std::shared_ptr<OrbKeyFrame>> OrbKeyFrame::GetVectorCovisibleKeyFrames()
{
    std::unique_lock<std::mutex> lock(m_connectionsMutex);
    return m_orderedConnectedKeyFrames;
}

std::vector<std::shared_ptr<OrbKeyFrame>> OrbKeyFrame::GetBestCovisibilityKeyFrames(const int &n)
{
    std::unique_lock<std::mutex> lock(m_connectionsMutex);
    if((int)m_orderedConnectedKeyFrames.size()<n)
    {
        return m_orderedConnectedKeyFrames;
    }
    else
    {
        return std::vector<std::shared_ptr<OrbKeyFrame>>(m_orderedConnectedKeyFrames.begin(),m_orderedConnectedKeyFrames.begin()+n);
    }

}

std::vector<std::shared_ptr<OrbKeyFrame>> OrbKeyFrame::GetCovisiblesByWeight(const int &weight)
{
    std::unique_lock<std::mutex> lock(m_connectionsMutex);

    if(m_orderedConnectedKeyFrames.empty())
    {
        return std::vector<std::shared_ptr<OrbKeyFrame>>();
    }

    std::vector<int>::iterator it = upper_bound(m_orderedWeights.begin(),m_orderedWeights.end(),weight,OrbKeyFrame::weightComp);
    if(it==m_orderedWeights.end())
    {
        return std::vector<std::shared_ptr<OrbKeyFrame>>();
    }
    else
    {
        int n = static_cast<int>(it - m_orderedWeights.begin());
        return std::vector<std::shared_ptr<OrbKeyFrame>>(m_orderedConnectedKeyFrames.begin(), m_orderedConnectedKeyFrames.begin()+n);
    }
}

int OrbKeyFrame::GetWeight(std::shared_ptr<OrbKeyFrame> keyFrame)
{
    std::unique_lock<std::mutex> lock(m_connectionsMutex);
    if(m_connectedKeyFrameWeights.count(keyFrame))
    {
        return m_connectedKeyFrameWeights[keyFrame];
    }
    else
    {
        return 0;
    }
}

void OrbKeyFrame::AddMapPoint(std::shared_ptr<OrbMapPoint> mapPoint, const size_t &idx)
{
    std::unique_lock<std::mutex> lock(m_featuresMutex);
    m_mapPoints[idx]=mapPoint;
}

void OrbKeyFrame::EraseMapPointMatch(const size_t &idx)
{
    std::unique_lock<std::mutex> lock(m_featuresMutex);
    m_mapPoints[idx]=static_cast<std::shared_ptr<OrbMapPoint>>(NULL);
}

void OrbKeyFrame::EraseMapPointMatch(std::shared_ptr<OrbMapPoint> mapPoint)
{
    int idx = mapPoint->GetObeservationIndexOfKeyFrame(shared_from_this());
    if(idx>=0)
    {
        m_mapPoints[idx]=static_cast<std::shared_ptr<OrbMapPoint>>(NULL);
    }
}


void OrbKeyFrame::ReplaceMapPointMatch(const size_t &idx, std::shared_ptr<OrbMapPoint> mapPoint)
{
    m_mapPoints[idx]=mapPoint;
}

std::set<std::shared_ptr<OrbMapPoint>> OrbKeyFrame::GetMapPoints()
{
    std::unique_lock<std::mutex> lock(m_featuresMutex);
    std::set<std::shared_ptr<OrbMapPoint>> s;
    for(size_t i=0, iend=m_mapPoints.size(); i<iend; i++)
    {
        if(!m_mapPoints[i])
        {
            continue;
        }
        std::shared_ptr<OrbMapPoint> pMP = m_mapPoints[i];
        if(!pMP->IsCorrupt())
        {
            s.insert(pMP);
        }
    }
    return s;
}

int OrbKeyFrame::TrackedMapPoints(const int &minObs)
{
    std::unique_lock<std::mutex> lock(m_featuresMutex);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        std::shared_ptr<OrbMapPoint> pMP = m_mapPoints[i];
        if(pMP)
        {
            if(!pMP->IsCorrupt())
            {
                if(bCheckObs)
                {
                    if(m_mapPoints[i]->GetObservingKeyFrameCount()>=minObs)
                    {
                        nPoints++;
                    }
                }
                else
                {
                    nPoints++;
                }
            }
        }
    }

    return nPoints;
}

std::vector<std::shared_ptr<OrbMapPoint>> OrbKeyFrame::GetMapPointMatches()
{
    std::unique_lock<std::mutex> lock(m_featuresMutex);
    return m_mapPoints;
}

std::shared_ptr<OrbMapPoint> OrbKeyFrame::GetMapPoint(const size_t &idx)
{
    std::unique_lock<std::mutex> lock(m_featuresMutex);
    return m_mapPoints[idx];
}

void OrbKeyFrame::UpdateConnections()
{
    std::map<std::shared_ptr<OrbKeyFrame>,int> KFcounter;
    std::vector<std::shared_ptr<OrbMapPoint>> vpMP;

    {
        std::unique_lock<std::mutex> lockMPs(m_featuresMutex);
        vpMP = m_mapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(std::vector<std::shared_ptr<OrbMapPoint>>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        std::shared_ptr<OrbMapPoint> pMP = *vit;

        if(!pMP)
        {
            continue;
        }

        if(pMP->IsCorrupt())
        {
            continue;
        }

        std::map<std::shared_ptr<OrbKeyFrame>,size_t> observations = pMP->GetObservingKeyframes();

        for(std::map<std::shared_ptr<OrbKeyFrame>,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId)
            {
                continue;
            }
            KFcounter[mit->first]++;
        }
    }
    // This should not happen
    if(KFcounter.empty())
    {
        return;
    }

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    std::shared_ptr<OrbKeyFrame> pKFmax;
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
            (mit->first)->AddConnection(std::shared_ptr<OrbKeyFrame>(shared_from_this()), mit->second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(shared_from_this(),nmax);
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
        std::unique_lock<std::mutex> lockCon(m_connectionsMutex);

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        m_connectedKeyFrameWeights = KFcounter;
        m_orderedConnectedKeyFrames = std::vector<std::shared_ptr<OrbKeyFrame>>(lKFs.begin(),lKFs.end());
        std::cout << lWs.size() << std::endl;
        m_orderedWeights = std::vector<int>(lWs.begin(), lWs.end());

        if(m_isFirstConnection && mnId!=0)
        {
            m_parent = m_orderedConnectedKeyFrames.front();
            m_parent->AddChild(shared_from_this());
            m_isFirstConnection = false;
        }

    }
}

void OrbKeyFrame::AddChild(std::shared_ptr<OrbKeyFrame> keyFrame)
{
    std::unique_lock<std::mutex> lockCon(m_connectionsMutex);
    m_children.insert(keyFrame);
}

void OrbKeyFrame::EraseChild(std::shared_ptr<OrbKeyFrame> keyFrame)
{
    std::unique_lock<std::mutex> lockCon(m_connectionsMutex);
    m_children.erase(keyFrame);
}

void OrbKeyFrame::ChangeParent(std::shared_ptr<OrbKeyFrame> keyFrame)
{
    std::unique_lock<std::mutex> lockCon(m_connectionsMutex);
    m_parent = keyFrame;
    keyFrame->AddChild(shared_from_this());
}

std::set<std::shared_ptr<OrbKeyFrame>> OrbKeyFrame::GetChilds()
{
    std::unique_lock<std::mutex> lockCon(m_connectionsMutex);
    return m_children;
}

std::shared_ptr<OrbKeyFrame> OrbKeyFrame::GetParent()
{
    std::unique_lock<std::mutex> lockCon(m_connectionsMutex);
    return m_parent;
}

bool OrbKeyFrame::hasChild(std::shared_ptr<OrbKeyFrame> keyFrame)
{
    std::unique_lock<std::mutex> lockCon(m_connectionsMutex);
    return static_cast<bool>(m_children.count(keyFrame));
}

void OrbKeyFrame::AddLoopEdge(std::shared_ptr<OrbKeyFrame> keyFrame)
{
    std::unique_lock<std::mutex> lockCon(m_connectionsMutex);
    m_shoulNotBeErased = true;
    m_loopEdges.insert(keyFrame);
}

std::set<std::shared_ptr<OrbKeyFrame>> OrbKeyFrame::GetLoopEdges()
{
    std::unique_lock<std::mutex> lockCon(m_connectionsMutex);
    return m_loopEdges;
}

void OrbKeyFrame::SetNotErase()
{
    std::unique_lock<std::mutex> lock(m_connectionsMutex);
    m_shoulNotBeErased = true;
}

void OrbKeyFrame::SetErase()
{
    {
        std::unique_lock<std::mutex> lock(m_connectionsMutex);
        if(m_loopEdges.empty())
        {
            m_shoulNotBeErased = false;
        }
    }

    if(m_shouldBeErased)
    {
        SetBadFlag();
    }
}

void OrbKeyFrame::SetBadFlag()
{
    {
        std::unique_lock<std::mutex> lock(m_connectionsMutex);
        if(mnId==0)
        {
            return;
        }
        else if(m_shoulNotBeErased)
        {
            m_shouldBeErased = true;
            return;
        }
    }

    for(std::map<std::shared_ptr<OrbKeyFrame>,int>::iterator mit = m_connectedKeyFrameWeights.begin(), mend=m_connectedKeyFrameWeights.end(); mit!=mend; mit++)
        mit->first->EraseConnection(shared_from_this());

    for(size_t i=0; i<m_mapPoints.size(); i++)
    {
        if(m_mapPoints[i])
        {
            m_mapPoints[i]->EraseObservingKeyframe(shared_from_this());
        }
    }

    {
        std::unique_lock<std::mutex> lock(m_connectionsMutex);
        std::unique_lock<std::mutex> lock1(m_featuresMutex);

        m_connectedKeyFrameWeights.clear();
        m_orderedConnectedKeyFrames.clear();

        // Update Spanning Tree
        std::set<std::shared_ptr<OrbKeyFrame>> sParentCandidates;
        sParentCandidates.insert(m_parent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while(!m_children.empty())
        {
            bool bContinue = false;

            int max = -1;
            std::shared_ptr<OrbKeyFrame> pC;
            std::shared_ptr<OrbKeyFrame> pP;

            for(std::set<std::shared_ptr<OrbKeyFrame>>::iterator sit=m_children.begin(), send=m_children.end(); sit!=send; sit++)
            {
                std::shared_ptr<OrbKeyFrame> pKF = *sit;
                if(pKF->isBad())
                {
                    continue;
                }

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
                m_children.erase(pC);
            }
            else
            {
                break;
            }
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!m_children.empty())
        {
            for(std::set<std::shared_ptr<OrbKeyFrame>>::iterator sit=m_children.begin(); sit!=m_children.end(); sit++)
            {
                (*sit)->ChangeParent(m_parent);
            }
        }

        m_parent->EraseChild(shared_from_this());
        mTcp = m_cameraPose*m_parent->GetPoseInverse();
        m_isBad = true;
    }


    m_map->DeleteOrbKeyFrame(shared_from_this());
    m_keyFrameDatabase->erase(shared_from_this());
}

bool OrbKeyFrame::isBad()
{
    std::unique_lock<std::mutex> lock(m_connectionsMutex);
    return m_isBad;
}

void OrbKeyFrame::EraseConnection(std::shared_ptr<OrbKeyFrame> keyFrame)
{
    bool bUpdate = false;
    {
        std::unique_lock<std::mutex> lock(m_connectionsMutex);
        if(m_connectedKeyFrameWeights.count(keyFrame))
        {
            m_connectedKeyFrameWeights.erase(keyFrame);
            bUpdate=true;
        }
    }

    if(bUpdate)
    {
        UpdateBestCovisibles();
    }
}

std::vector<size_t> OrbKeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    std::vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = std::max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
    {
        return vIndices;
    }

    const int nMaxCellX = std::min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
    {
        return vIndices;
    }

    const int nMinCellY = std::max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
    {
        return vIndices;
    }

    const int nMaxCellY = std::min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
    {
        return vIndices;
    }

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const std::vector<size_t> vCell = m_grid[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                {
                    vIndices.push_back(vCell[j]);
                }
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

        std::unique_lock<std::mutex> lock(m_poseMutex);
        return m_reverseCameraPose.rowRange(0,3).colRange(0,3)*x3Dc+m_reverseCameraPose.rowRange(0,3).col(3);
    }
    else
    {
        return cv::Mat();
    }
}

float OrbKeyFrame::ComputeSceneMedianDepth(const int q)
{
    std::vector<std::shared_ptr<OrbMapPoint>> mapPoints;
    cv::Mat Tcw_;
    {
        std::unique_lock<std::mutex> lock(m_featuresMutex);
        std::unique_lock<std::mutex> lock2(m_poseMutex);
        mapPoints = m_mapPoints;
        Tcw_ = m_cameraPose.clone();
    }

    std::vector<float> vDepths;
    vDepths.reserve(N);
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    for(int i=0; i<N; i++)
    {
        if(m_mapPoints[i])
        {
            std::shared_ptr<OrbMapPoint> pMP = m_mapPoints[i];
            cv::Mat x3Dw = pMP->GetWorldPosition();
            float z = static_cast<float>(Rcw2.dot(x3Dw) + zcw);
            vDepths.push_back(z);
        }
    }

    std::sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}