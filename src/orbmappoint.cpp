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

#include <orbmappoint.hpp>
#include <orbmatcher.hpp>
long unsigned int OrbMapPoint::nNextId=0;

OrbMapPoint::OrbMapPoint(const cv::Mat &position, std::shared_ptr<OrbKeyFrame> refenceKeyFrame, std::shared_ptr<OrbMap> map)
    : m_firstKeyframeId(refenceKeyFrame->m_id), m_FirstKeyFrame(refenceKeyFrame->m_id),
      m_refenceKeyFrame(refenceKeyFrame), m_map(map)
{
    position.copyTo(m_worldPosition);
    m_meanViewingDirection = cv::Mat::zeros(3, 1, CV_32F);
    m_constructorTag = 0;
    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    std::unique_lock<std::mutex> lock(m_constructorMutex);
    m_sequenceId = nNextId++;
}

OrbMapPoint::OrbMapPoint(const cv::Mat &position, std::shared_ptr<OrbKeyFrame> frame, std::shared_ptr<OrbMap> map, const int &keyPointIndex)
    : m_map(map)
{
    position.copyTo(m_worldPosition);
    cv::Mat cameraCenter = frame->GetCameraCenter();
    m_meanViewingDirection = m_worldPosition - cameraCenter;
    m_meanViewingDirection = m_meanViewingDirection / cv::norm(m_meanViewingDirection);
    m_constructorTag = 1;
    cv::Mat offset = position - cameraCenter;
    const float distance = (float)cv::norm(offset);
    const int level = frame->mvKeysUn[keyPointIndex].octave;
    const float levelScaleFactor = frame->mvScaleFactors[level];
    const int levels = frame->mnScaleLevels;

    m_maxDistance = distance*levelScaleFactor;
    m_minDistance = m_maxDistance/frame->mvScaleFactors[levels-1];

    frame->mDescriptors.row(keyPointIndex).copyTo(m_descriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    std::unique_lock<std::mutex> lock(m_constructorMutex);
    m_sequenceId = nNextId++;
}

OrbMapPoint::OrbMapPoint(const cv::Mat &position, std::shared_ptr<OrbFrame> frame, std::shared_ptr<OrbMap> map, const int &keyPointIndex)
        : m_map(map)
{
    position.copyTo(m_worldPosition);
    cv::Mat cameraCenter = frame->GetCameraCenter();
    m_meanViewingDirection = m_worldPosition - cameraCenter;
    m_meanViewingDirection = m_meanViewingDirection / cv::norm(m_meanViewingDirection);

    cv::Mat offset = position - cameraCenter;
    const float distance = (float)cv::norm(offset);
    const int level = frame->m_undistortedKeys[keyPointIndex].octave;
    const float levelScaleFactor = frame->m_scaleFactors[level];
    const int levels = frame->m_scaleLevels;
    m_constructorTag = 2;
    m_maxDistance = distance*levelScaleFactor;
    m_minDistance = m_maxDistance/frame->m_scaleFactors[levels-1];

    frame->m_descriptors.row(keyPointIndex).copyTo(m_descriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    std::unique_lock<std::mutex> lock(m_constructorMutex);
    m_sequenceId = nNextId++;
}

OrbMapPoint::~OrbMapPoint()
{}

std::map<std::shared_ptr<OrbKeyFrame>,size_t> OrbMapPoint::GetObservingKeyframes()
{
    // use proper mutex
    std::unique_lock<std::mutex> lock(m_featureMutex);
    return m_observingKeyframes;
}

bool OrbMapPoint::IsCorrupt()
{
    std::unique_lock<std::mutex> featureMutex(this->m_featureMutex);
    std::unique_lock<std::mutex> positionMutex(this->m_positionMutex);
    return this->m_corrupt;
}

// src/orboptimizer.cpp
// 233:            pMP->SetWorldPosition(Orbconverter::toCvMat(vPoint->estimate()));
// 787:        pMP->SetWorldPosition(Orbconverter::toCvMat(vPoint->estimate()));
void OrbMapPoint::SetWorldPosition(const cv::Mat &position)
{
    // use proper mutex here
    position.copyTo(m_worldPosition);
}
// src/sim3solver.cpp
// 98:            cv::Mat X3D1w = mapPoint1->GetWorldPosition();
// 101:            cv::Mat X3D2w = mapPoint2->GetWorldPosition();

// src/orboptimizer.cpp
// 96:        vPoint->setEstimate(Orbconverter::toVector3d(pMP->GetWorldPosition()));
// 318:                cv::Mat Xw = pMP->GetWorldPosition();
// 356:                cv::Mat Xw = pMP->GetWorldPosition();
// 582:        vPoint->setEstimate(Orbconverter::toVector3d(pMP->GetWorldPosition()));
// 861:                cv::Mat P3D1w = pMP1->GetWorldPosition();
// 869:                cv::Mat P3D2w = pMP2->GetWorldPosition();
cv::Mat OrbMapPoint::GetWorldPosition()
{
    // use proper mutex
    return m_worldPosition.clone();
}

cv::Mat OrbMapPoint::GetMeanViewingDirection()
{
    std::unique_lock<std::mutex> lock(m_positionMutex);
    return m_meanViewingDirection;
}

std::shared_ptr<OrbKeyFrame> OrbMapPoint::GetReferenceKeyFrame() {
    std::unique_lock<std::mutex> lock(m_featureMutex);
    return m_refenceKeyFrame;
}

// src/orbframe.cpp
// 416:                    if(m_mapPoints[i]->GetObservingKeyFrameCount()>=minimumObservations)
int OrbMapPoint::GetObservingKeyFrameCount()
{
    // use mutex
    return m_observingKeyFramesCount;
}

int OrbMapPoint::GetSequenceId()
{
    return m_sequenceId;
}

void OrbMapPoint::AddObservingKeyframe(std::shared_ptr<OrbKeyFrame> keyFrame, size_t idx)
{
    if (keyFrame.get() && idx)
    {
        std::unique_lock<std::mutex> lock(this->m_featureMutex);
        if(this->m_observingKeyframes.count(keyFrame))
            return;
        this->m_observingKeyframes[keyFrame]=idx;
        if(keyFrame->mvuRight[idx]>=0)
            m_observingKeyFramesCount+=2;
        else
            m_observingKeyFramesCount++;
    }
}
// src/orboptimizer.cpp
// 767:            pMPi->EraseObservingKeyframe(keyFramei);
void OrbMapPoint::EraseObservingKeyframe(std::shared_ptr<OrbKeyFrame> keyFrame)
{
    std::unique_lock<std::mutex> lock(m_featureMutex);
    // check if OrbKeyFrame in m_observingKeyframes
    if (this->m_observingKeyframes.count(keyFrame))
    {
        int keyFrameId = this->m_observingKeyframes[keyFrame];
        // determine if monocular or stereo keyframe
        if(keyFrame->mvuRight[keyFrameId]>=0)
            this->m_observingKeyFramesCount -= 2;
        else
            this->m_observingKeyFramesCount -= 1;

        if (m_refenceKeyFrame == keyFrame)
            m_refenceKeyFrame = this->m_observingKeyframes.begin()->first;
        
        this->m_observingKeyframes.erase(keyFrame);
        if (this->m_observingKeyFramesCount <= 2){
            lock.unlock();
            SetCorruptFlag();
        }
    }
}
// src/orbframe.cpp
// 383:    int index = mapPoint->GetObeservationIndexOfKeyFrame(std::shared_ptr<OrbFrame>(this));

// src/sim3solver.cpp
// 69:            int indexKF1 = mapPoint1->GetObeservationIndexOfKeyFrame(localKeyFrame1);
// 70:            int indexKF2 = mapPoint2->GetObeservationIndexOfKeyFrame(localKeyFrame2);

// src/orboptimizer.cpp
// 854:        const int i2 = pMP2->GetObeservationIndexOfKeyFrame(keyFrame2);
int OrbMapPoint::GetObeservationIndexOfKeyFrame(std::shared_ptr<OrbKeyFrame> keyFrame)
{
    std::unique_lock<std::mutex> lock(m_featureMutex);
    if (this->m_observingKeyframes.count(keyFrame))
        return this->m_observingKeyframes[keyFrame];
    else
        return -1;
}

bool OrbMapPoint::KeyFrameInObservingKeyFrames(std::shared_ptr<OrbKeyFrame> keyFrame)
{
    if (keyFrame.get())
    {
        std::unique_lock<std::mutex> lock(m_featureMutex);
        return (m_observingKeyframes.count(keyFrame));
    }
    return false;
}

void OrbMapPoint::SetCorruptFlag()
{
     std::map<std::shared_ptr<OrbKeyFrame>,size_t> obs;
    {
        std::unique_lock<std::mutex> lock1(m_featureMutex);
        std::unique_lock<std::mutex> lock2(m_positionMutex);
        m_corrupt=true;
        obs = m_observingKeyframes;
        m_observingKeyframes.clear();
    }
    for(std::map<std::shared_ptr<OrbKeyFrame>,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        std::shared_ptr<OrbKeyFrame> keyFrame = mit->first;
        keyFrame->EraseMapPointMatch(mit->second);
    }

    m_map->DeleteOrbMapPoint(std::shared_ptr<OrbMapPoint>(shared_from_this()));
}

void OrbMapPoint::Replace(std::shared_ptr<OrbMapPoint> orbMapPoint)
{
    if (orbMapPoint != 0)
    {
        if(orbMapPoint->GetSequenceId()==this->GetSequenceId())
            return;

        int nvisible, nfound;
        std::map<std::shared_ptr<OrbKeyFrame>,size_t> obs;
        {
            std::unique_lock<std::mutex> lock1(m_featureMutex);
            std::unique_lock<std::mutex> lock2(m_positionMutex);
            obs=m_observingKeyframes;
            m_observingKeyframes.clear();
            m_corrupt=true;
            nvisible = m_visibleCounter;
            nfound = m_foundCounter;
            m_replaced = orbMapPoint;
        }

        for(std::map<std::shared_ptr<OrbKeyFrame>,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
        {
        // Replace measurement in keyframe
            std::shared_ptr<OrbKeyFrame> keyFrame = mit->first;

            if(!orbMapPoint->KeyFrameInObservingKeyFrames(keyFrame))
            {
                keyFrame->ReplaceMapPointMatch(mit->second, orbMapPoint);
                orbMapPoint->AddObservingKeyframe(keyFrame,mit->second);
            }
            else
            {
                keyFrame->EraseMapPointMatch(mit->second);
            }
        }
        orbMapPoint->IncreaseFound(nfound);
        orbMapPoint->IncreaseVisible(nvisible);
        orbMapPoint->ComputeDistinctiveDescriptors();
        m_map->DeleteOrbMapPoint(std::shared_ptr<OrbMapPoint>(shared_from_this()));
    }
}

std::shared_ptr<OrbMapPoint> OrbMapPoint::GetReplaced()
{
    std::unique_lock<std::mutex> lock1(m_featureMutex);
    std::unique_lock<std::mutex> lock2(m_positionMutex);
    return m_replaced;
}

void OrbMapPoint::IncreaseVisible(int n)
{
    if (n)
    {
        std::unique_lock<std::mutex> lock(m_featureMutex);
        m_visibleCounter+=n;
    }
}

void OrbMapPoint::IncreaseFound(int n)
{
    if (n)
    {
        std::unique_lock<std::mutex> lock(m_featureMutex);
        m_foundCounter+=n;
    }
}

float OrbMapPoint::GetFoundRatio()
{
    std::unique_lock<std::mutex> lock(m_featureMutex);
    return static_cast<float>(m_foundCounter)/m_visibleCounter;
}

void OrbMapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    std::unique_lock<std::mutex> featureMutex(this->m_featureMutex);
    if (this->m_corrupt)
        return;
    std::map<std::shared_ptr<OrbKeyFrame>, size_t> observingKeyFrames = this->m_observingKeyframes;
    //featureMutex.unlock();
    if (observingKeyFrames.empty())
    {
        return;
    }
    std::vector<cv::Mat> descriptors;
    descriptors.reserve(observingKeyFrames.size());

    for (std::map<std::shared_ptr<OrbKeyFrame>, size_t>::iterator iteratorBegin = observingKeyFrames.begin(), iteratorEnd = observingKeyFrames.end(); iteratorBegin != iteratorEnd; iteratorBegin++)
    {
        std::shared_ptr<OrbKeyFrame> keyFrame = iteratorBegin->first;

        if (!keyFrame->isBad())
        {
            descriptors.push_back(keyFrame->mDescriptors.row(static_cast<int>(iteratorBegin->second)));
        }
    }

    if (descriptors.empty())
    {
        return;
    }
    const size_t N = descriptors.size();
    float **distances = new float *[N];

    for (size_t i = 0; i < N; ++i)
    {
        distances[i] = new float[N];
    }
    for (size_t i = 0; i < N; i++)
    {
        distances[i][i] = 0;
        for (size_t j = i + 1; j < N; j++)
        {
            int distance = ORBmatcher::DescriptorDistance(descriptors[i], descriptors[j]);
            distances[i][j] = distance;
            distances[j][i] = distance;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for (size_t i = 0; i < N; i++)
    {
        std::vector<int> dists(distances[i], distances[i] + N);
        sort(dists.begin(), dists.end());
        int median = dists[static_cast<int>(0.5 * (static_cast<float>(N) - 1.0))];

        if (median < BestMedian)
        {
            BestMedian = median;
            BestIdx = static_cast<int>(i);
        }
    }
    for (size_t i = 0; i < N; ++i)
    {
        delete distances[i];
    }
    delete distances;
    //featureMutex.lock();
    this->m_descriptor = descriptors[BestIdx].clone();
}
// include/orbframe.hpp
// 97:    cv::Mat GetDescriptors() { return m_descriptors; }
cv::Mat OrbMapPoint::GetDescriptor()
{
    std::unique_lock<std::mutex> lock(m_featureMutex);
    return m_descriptor.clone();
}
// src/orboptimizer.cpp
// 234:            pMP->UpdateMeanAndDepthValues();
// 788:        pMP->UpdateMeanAndDepthValues();
void OrbMapPoint::UpdateMeanAndDepthValues()
{
    // aquire locks
    std::unique_lock<std::mutex> featureMutex(this->m_featureMutex);
    std::unique_lock<std::mutex> positionMutex(this->m_positionMutex);
    if (this->m_corrupt)
        return;
    std::map<std::shared_ptr<OrbKeyFrame>, size_t> observingKeyFrames = this->m_observingKeyframes;
    std::shared_ptr<OrbKeyFrame> referenceKeyFrame = this->m_refenceKeyFrame;
    cv::Mat position = this->m_worldPosition.clone();
    featureMutex.unlock();
    positionMutex.unlock();
    // release locks
    if (observingKeyFrames.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(std::map<std::shared_ptr<OrbKeyFrame>, size_t>::iterator iteratorBegin=observingKeyFrames.begin(), iteratorEnd=observingKeyFrames.end(); iteratorBegin!=iteratorEnd; iteratorBegin++)
    {
        std::shared_ptr<OrbKeyFrame> keyFrame = iteratorBegin->first;
        cv::Mat frameCameraCenter = keyFrame->GetCameraCenter();
        cv::Mat normali = this->m_worldPosition - frameCameraCenter;
        normal = normal + normali / cv::norm(normali);
        n++;
    }

    cv::Mat PC = position - referenceKeyFrame->GetCameraCenter();
    const float dist = static_cast<float>(cv::norm(PC));
    const int level = referenceKeyFrame->mvKeysUn[observingKeyFrames[referenceKeyFrame]].octave;
    const float levelScaleFactor =  referenceKeyFrame->mvScaleFactors[level];
    const int nLevels = referenceKeyFrame->mnScaleLevels;

    positionMutex.lock();
    this->m_maxDistance = dist*levelScaleFactor;
    this->m_minDistance = m_maxDistance/referenceKeyFrame->mvScaleFactors[nLevels-1];
    this->m_meanViewingDirection = normal/n;
}

float OrbMapPoint::GetMinDistanceInvariance()
{
    std::unique_lock<std::mutex> lock(m_positionMutex);
    return 0.8f*m_minDistance;
}

float OrbMapPoint::GetMaxDistanceInvariance()
{
    std::unique_lock<std::mutex> lock(m_positionMutex);
    return 1.2f*m_maxDistance;
}

int OrbMapPoint::PredictScale(const float &currentDist, std::shared_ptr<OrbKeyFrame> keyFrame) {
    float ratio;
    {
        std::unique_lock<std::mutex> lock(m_positionMutex);
        ratio = m_maxDistance/currentDist;
    }

    int nScale = static_cast<int>(ceil(log(ratio) / keyFrame->mfLogScaleFactor));
    if(nScale<0)
        nScale = 0;
    else if(nScale>=keyFrame->mnScaleLevels)
        nScale = keyFrame->mnScaleLevels-1;

    return nScale;
}

int OrbMapPoint::PredictScale(const float &currentDist, std::shared_ptr<OrbFrame> pF)
{
    float ratio;
    {
        std::unique_lock<std::mutex> lock(m_positionMutex);
        ratio = m_maxDistance/currentDist;
    }

    int nScale = static_cast<int>(ceil(log(ratio) / pF->m_logScaleFactor));
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->m_scaleLevels)
        nScale = pF->m_scaleLevels-1;

    return nScale;
}

int OrbMapPoint::GetTrackScaleLevel()
{
    return mnTrackScaleLevel;
}

bool OrbMapPoint::GetTrackInView()
{
    return mbTrackInView;
}

float OrbMapPoint::GTrackViewCos()
{
    return mTrackViewCos;
}

long unsigned int OrbMapPoint::GetTrackReferenceForFrame()
{
    return mnTrackReferenceForFrame;
}

long unsigned int OrbMapPoint::GetLastFrameSeen()
{
    return mnLastFrameSeen;
}

// include/orbframe.hpp
// 124:    long unsigned int GetBALocalForKF(){return mnBALocalForKF; }

// src/orboptimizer.cpp
// 491:                    if(pMP->GetBALocalForKF()!=keyFrame->Id)
// 508:            if(keyFramei->GetBALocalForKF()!=keyFrame->Id && keyFramei->GetBAFixedForKF()!=keyFrame->Id)

long unsigned int OrbMapPoint::GetBALocalForKF()
{
    return this->mnBALocalForKF;
}

long unsigned int OrbMapPoint::GetFuseCandidateForKF()
{
    return this->mnFuseCandidateForKF;
}

long unsigned int OrbMapPoint::GetLoopPointForKF()
{
    return this->mnLoopPointForKF;
}

long unsigned int OrbMapPoint::GetCorrectedByKF()
{
    return this->mnCorrectedByKF;
}

long unsigned int OrbMapPoint::GetCorrectedReference()
{
    return this->mnCorrectedReference;
}
/*
cv::Mat OrbMapPoint::GetPosGBA()
{
    return cv::Mat();
}

long unsigned int OrbMapPoint::GetBAGlobalForKF()
{
    return 0;
}

void OrbMapPoint::SetTrackScaleLevel(long unsigned int TrackScaleLevel)
{
    if (TrackScaleLevel != 0)
    {
    };
}
*/
void OrbMapPoint::SetTrackInView(bool TrackInView)
{
    mbTrackInView = TrackInView;
}
/*
void OrbMapPoint::SetackViewCos(long unsigned int ackViewCos)
{
    if (ackViewCos != 0)
    {
    };
}
*/
void OrbMapPoint::SetTrackReferenceForFrame(long unsigned int TrackReferenceForFrame)
{
    if (TrackReferenceForFrame != 0)
    {
        mnTrackReferenceForFrame = TrackReferenceForFrame;
    };
}

void OrbMapPoint::SetLastFrameSeen(long unsigned int LastFrameSeen)
{
    if (LastFrameSeen != 0)
    {
        mnLastFrameSeen=LastFrameSeen;
    };
}
// include/orbframe.hpp

void OrbMapPoint::SetTrackProjX(const float d)
{
    mTrackProjX = d;
}

void OrbMapPoint::SetTrackProjXR(float d)
{
    mTrackProjXR = d;
}

void OrbMapPoint::SetTrackProjY(const float d)
{
    mTrackProjY = d;
}

void OrbMapPoint::SetnTrackScaleLevel(const int i)
{
    mnTrackScaleLevel = i;
}

void OrbMapPoint::SetTrackViewCos(const float d)
{
    mTrackViewCos = d;
}

// 125:    void SetBALocalForKF(long unsigned int inBALocal){ mnBALocalForKF = inBALocal; }

// src/orboptimizer.cpp
// 470:    keyFrame->SetBALocalForKF(keyFrame->Id);
// 476:        keyFramei->SetBALocalForKF(keyFrame->Id);;
// 494:                        pMP->SetBALocalForKF(keyFrame->Id);
void OrbMapPoint::SetBALocalForKF(long unsigned int BALocalForKF)
{
    // use mutex
    this->mnBALocalForKF = BALocalForKF;
}

void OrbMapPoint::SetFuseCandidateForKF(long unsigned int FuseCandidateForKF)
{
    if (FuseCandidateForKF != 0)
    {
        mnFuseCandidateForKF=FuseCandidateForKF;
    };
}

void OrbMapPoint::SetLoopPointForKF(long unsigned int LoopPointForKF)
{
    if (LoopPointForKF != 0)
    {
        mnLoopPointForKF = LoopPointForKF;
    };
}

void OrbMapPoint::SetCorrectedByKF(long unsigned int CorrectedByKF)
{
    if (CorrectedByKF != 0)
    {
        mnCorrectedByKF = CorrectedByKF;
    };
}

void OrbMapPoint::SetCorrectedReference(long unsigned int CorrectedReference)
{
    if (CorrectedReference != 0)
    {
        mnCorrectedReference = CorrectedReference;
    };
}
/*
void OrbMapPoint::SetPosGBA(long unsigned int PosGBA)
{
    if (PosGBA != 0)
    {
    };
}

void OrbMapPoint::SetBAGlobalForKF(long unsigned int BAGlobalForKF)
{
    if (BAGlobalForKF != 0)
    {
    };
}
*/