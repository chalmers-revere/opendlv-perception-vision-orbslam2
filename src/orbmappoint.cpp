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

#include <orbmappoint.hpp>

OrbMapPoint::OrbMapPoint(const cv::Mat &position, std::shared_ptr<OrbFrame> refenceKeyFrame, std::shared_ptr<OrbMap> map)
: Id(), m_nextId(), m_firstKeyframeId(refenceKeyFrame->Id), m_FirstKeyFrame(refenceKeyFrame->Id), m_refenceKeyFrame(refenceKeyFrame), m_map(map)
{
    position.copyTo(m_worldPosition);
    m_meanViewingDirection = cv::Mat::zeros(3,1,CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    std::unique_lock<std::mutex> lock(m_constructorMutex);
    m_sequenceId=m_nextId++;
}

OrbMapPoint::OrbMapPoint(const cv::Mat &position, std::shared_ptr<OrbFrame> frame, std::shared_ptr<OrbMap> map, const int &keyPointIndex)
: Id(), m_nextId(), m_map(map)
{
    position.copyTo(m_worldPosition);
    cv::Mat cameraCenter = frame->GetCameraCenter();
    m_meanViewingDirection = m_worldPosition - cameraCenter;
    m_meanViewingDirection = m_meanViewingDirection/cv::norm(m_meanViewingDirection);

    cv::Mat offset = position - cameraCenter;
    const float distance = (float)cv::norm(offset);
    const int level = frame->GetUndistortedKeyPoints()[keyPointIndex].octave;
    const float levelScaleFactor = frame->GetScaleFactors()[level];
    const int levels = frame->GetScaleLevels();

    m_maxDistance = distance*levelScaleFactor;
    m_minDistance = m_maxDistance/frame->GetScaleFactors()[levels-1];

    frame->GetDescriptors().row(keyPointIndex).copyTo(m_descriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    std::unique_lock<std::mutex> lock(m_constructorMutex);
    m_sequenceId=m_nextId++;
}

OrbMapPoint::~OrbMapPoint()
{

}

std::map<std::shared_ptr<OrbFrame>,size_t> OrbMapPoint::GetObservingKeyframes()
{
    // use proper mutex
    return m_observingKeyframes;
}

bool OrbMapPoint::IsCorrupt()
{
    return false;
}

// src/orboptimizer.cpp
// 233:            pMP->SetWorldPosition(Orbconverter::toCvMat(vPoint->estimate()));
// 787:        pMP->SetWorldPosition(Orbconverter::toCvMat(vPoint->estimate()));
void OrbMapPoint::SetWorldPosition(const cv::Mat &Position) {
    // use proper mutex here
    pos.copyTo(m_worldPosition);
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
cv::Mat OrbMapPoint::GetWorldPosition() {
    // use proper mutex
    return m_worldPosition.copy();
}

cv::Mat OrbMapPoint::GetMeanViewingDirection() {
    return cv::Mat();
}

std::shared_ptr<OrbFrame> OrbMapPoint::GetReferenceKeyFrame() {
    return std::shared_ptr<OrbFrame>();
}
// src/orbframe.cpp
// 416:                    if(m_mapPoints[i]->GetObservingKeyFrameCount()>=minimumObservations)
int OrbMapPoint::GetObservingKeyFrameCount() {
    // use mutex
    return m_observingKeyFramesCount;
}

int OrbMapPoint::GetSequenceId() {
    return 0;
}

void OrbMapPoint::AddObservingKeyframe(std::shared_ptr<OrbFrame> keyFrame, size_t idx) {
    if(keyFrame.get() && idx) {}
}
// src/orboptimizer.cpp
// 767:            pMPi->EraseObservingKeyframe(pKFi);
void OrbMapPoint::EraseObservingKeyframe(std::shared_ptr<OrbFrame> keyFrame) {
    bool mapPointIsCorrupt = false;
    // aquire proper mutex

    // check if OrbKeyFrame in m_observingKeyframes
    if(this->m_observingKeyframes.count(keyFrame))
    {
        int keyFrameId = this->m_observingKeyframes[keyFrame];
        // determine if monocular or stereo keyframe
        if(keyFrame->mvuRight[keyFrameId]>=0)
            this->m_observingKeyFramesCount -= 2;
        else
            this->m_observingKeyFramesCount -= 1;

        this->m_observingKeyframes.erase(keyFrame);

        if(m_refenceKeyFrame == keyFrame)
            m_refenceKeyFrame = this->m_observingKeyframes.begin()->first;

        
        if(this->m_observingKeyFramesCount <= 2)
            // release mutex
            SetCorruptFlag();
    }
}
// src/orbframe.cpp
// 383:    int index = mapPoint->GetObeservationIndexOfKeyFrame(std::shared_ptr<OrbFrame>(this));

// src/sim3solver.cpp
// 69:            int indexKF1 = mapPoint1->GetObeservationIndexOfKeyFrame(localKeyFrame1);
// 70:            int indexKF2 = mapPoint2->GetObeservationIndexOfKeyFrame(localKeyFrame2);

// src/orboptimizer.cpp
// 854:        const int i2 = pMP2->GetObeservationIndexOfKeyFrame(pKF2);
int OrbMapPoint::GetObeservationIndexOfKeyFrame(std::shared_ptr<OrbFrame> keyFrame) {
    // use proper mutex
    if(this->m_observingKeyframes.count(keyFrame))
        return this->m_observingKeyframes[keyFrame];
    else
        return -1;
}

bool OrbMapPoint::KeyFrameInObservingKeyFrames(std::shared_ptr<OrbFrame> keyFrame) {
    if(keyFrame.get()) {}
    return false;
}

void OrbMapPoint::SetCorruptFlag() {

}

void OrbMapPoint::Replace(std::shared_ptr<OrbMapPoint> orbMapPoint) {
    if(orbMapPoint != 0){}
}

std::shared_ptr<OrbMapPoint> OrbMapPoint::GetReplaced() {
    return std::shared_ptr<OrbMapPoint>();
}

void OrbMapPoint::IncreaseVisible(int n) {
    if(n){}
}

void OrbMapPoint::IncreaseFound(int n) {
    if(n){}
}

float OrbMapPoint::GetFoundRatio() {
    return 0;
}

void OrbMapPoint::ComputeDistinctiveDescriptors() {

}
// include/orbframe.hpp
// 97:    cv::Mat GetDescriptors() { return m_descriptors; }
cv::Mat OrbMapPoint::GetDescriptor() {
    // use mutex
    return m_descriptor.clone();
}
// src/orboptimizer.cpp
// 234:            pMP->UpdateMeanAndDepthValues();
// 788:        pMP->UpdateMeanAndDepthValues();
void OrbMapPoint::UpdateMeanAndDepthValues() {
    if (this->m_corrupt) return;

    std::map<std::shared_ptr<OrbFrame>,size_t> observations;
    std::shared_ptr<OrbFrame> referenceKeyFrame;
    cv::Mat position;
    // aquire locks 
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    observations=this->m_observingKeyframes;
    pRefKF=mpRefKF;
    Pos = mWorldPos.clone();
    
    // release locks
    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        cv::Mat Owi = pKF->GetCameraCenter();
        cv::Mat normali = mWorldPos - Owi;
        normal = normal + normali/cv::norm(normali);
        n++;
    }

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
    }
}

float OrbMapPoint::GetMinDistanceInvariance() {
    return 0;
}

float OrbMapPoint::GetMaxDistanceInvariance() {
    return 0;
}

int OrbMapPoint::PredictScale(const float &currentDist, std::shared_ptr<OrbFrame> keyFrame) {
    if(sizeof(currentDist) == sizeof(float)){}
    keyFrame.get();
    return 0;
}

int OrbMapPoint::GetTrackScaleLevel() {
    return 0;
}

bool OrbMapPoint::GetTrackInView() {
    return false;
}

float OrbMapPoint::GTrackViewCos() {
    return 0;
}

long unsigned int OrbMapPoint::GetTrackReferenceForFrame() {
    return 0;
}

long unsigned int OrbMapPoint::GetLastFrameSeen() {
    return 0;
}
// include/orbframe.hpp
// 124:    long unsigned int GetBALocalForKF(){return mnBALocalForKF; }

// src/orboptimizer.cpp
// 491:                    if(pMP->GetBALocalForKF()!=pKF->Id)
// 508:            if(pKFi->GetBALocalForKF()!=pKF->Id && pKFi->GetBAFixedForKF()!=pKF->Id)
long unsigned int OrbMapPoint::GetBALocalForKF() {
    return 0;
}

long unsigned int OrbMapPoint::GetFuseCandidateForKF() {
    return 0;
}

long unsigned int OrbMapPoint::GetLoopPointForKF() {
    return 0;
}

long unsigned int OrbMapPoint::GetCorrectedByKF() {
    return 0;
}

long unsigned int OrbMapPoint::GetCorrectedReference() {
    return 0;
}

cv::Mat OrbMapPoint::GetPosGBA() {
    return cv::Mat();
}

long unsigned int OrbMapPoint::GetBAGlobalForKF() {
    return 0;
}

void OrbMapPoint::SetTrackScaleLevel(long unsigned int TrackScaleLevel) {
if (TrackScaleLevel != 0){};
}

void OrbMapPoint::SetTrackInView(long unsigned int TrackInView) {
if (TrackInView != 0){};
}

void OrbMapPoint::SetackViewCos(long unsigned int ackViewCos) {
if (ackViewCos != 0){};
}

void OrbMapPoint::SetTrackReferenceForFrame(long unsigned int TrackReferenceForFrame) {
if (TrackReferenceForFrame != 0){};
}

void OrbMapPoint::SetLastFrameSeen(long unsigned int LastFrameSeen) {
if (LastFrameSeen != 0){};
}
// include/orbframe.hpp
// 125:    void SetBALocalForKF(long unsigned int inBALocal){ mnBALocalForKF = inBALocal; }

// src/orboptimizer.cpp
// 470:    pKF->SetBALocalForKF(pKF->Id);
// 476:        pKFi->SetBALocalForKF(pKF->Id);;
// 494:                        pMP->SetBALocalForKF(pKF->Id);
void OrbMapPoint::SetBALocalForKF(long unsigned int BALocalForKF) {
if (BALocalForKF != 0){};
}

void OrbMapPoint::SetFuseCandidateForKF(long unsigned int FuseCandidateForKF) {
if (FuseCandidateForKF != 0){};
}

void OrbMapPoint::SetLoopPointForKF(long unsigned int LoopPointForKF) {
if (LoopPointForKF != 0){};
}

void OrbMapPoint::SetCorrectedByKF(long unsigned int CorrectedByKF) {
if (CorrectedByKF != 0){};
}

void OrbMapPoint::SetCorrectedReference(long unsigned int CorrectedReference) {
if (CorrectedReference != 0){};
}

void OrbMapPoint::SetPosGBA(long unsigned int PosGBA) {
if (PosGBA != 0){};
}

void OrbMapPoint::SetBAGlobalForKF(long unsigned int BAGlobalForKF) {
if (BAGlobalForKF != 0){};
}
