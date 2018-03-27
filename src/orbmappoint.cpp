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
namespace opendlv {
namespace logic {
namespace sensation {

OrbMapPoint::OrbMapPoint(const cv::Mat &position, std::shared_ptr<OrbFrame> refenceKeyFrame, std::shared_ptr<OrbMap> map)
: m_nextId(), m_firstKeyframeId(refenceKeyFrame->Id), m_FirstKeyFrame(refenceKeyFrame->Id), m_refenceKeyFrame(refenceKeyFrame), m_map(map)
{
    position.copyTo(m_worldPosition);
    m_meanViewingDirection = cv::Mat::zeros(3,1,CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    std::unique_lock<std::mutex> lock(m_constructorMutex);
    m_sequenceId=m_nextId++;
}

OrbMapPoint::OrbMapPoint(const cv::Mat &position, std::shared_ptr<OrbFrame> frame, std::shared_ptr<OrbMap> map, const int &keyPointIndex)
: m_nextId(), m_map(map)
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
    return m_observingKeyframes;
}

bool OrbMapPoint::IsCorrupt()
{
    return false;
}

void OrbMapPoint::SetWorldPosition(const cv::Mat &Position) {
    if(Position.empty()){}
}

cv::Mat OrbMapPoint::GetWorldPosition() {
    return cv::Mat();
}

    cv::Mat OrbMapPoint::GetMeanViewingDistance()
    {
        std::unique_lock<std::mutex> lock(mMutexPos);
        return m_meanViewingDirection.clone();
    }

cv::Mat OrbMapPoint::GetMeanViewingDirection() {
    return cv::Mat();
}

std::shared_ptr<OrbFrame> OrbMapPoint::GetReferenceKeyFrame() {
    return std::shared_ptr<OrbFrame>();
}

int OrbMapPoint::GetObservingKeyFrameCount() {
    return 0;
}

int OrbMapPoint::GetSequenceId() {
    return 0;
}

void OrbMapPoint::AddObservingKeyframe(std::shared_ptr<OrbFrame> keyFrame, size_t idx) {
    if(keyFrame.get() && idx) {}
}

void OrbMapPoint::EraseObservingKeyframe(std::shared_ptr<OrbFrame> keyFrame) {
    if(keyFrame.get()) {}
}

int OrbMapPoint::GetObeservationIndexOfKeyFrame(std::shared_ptr<OrbFrame> keyFrame) {
    if(keyFrame.get()) {}
    return 0;
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

cv::Mat OrbMapPoint::GetDescriptor() {
    return cv::Mat();
}

void OrbMapPoint::UpdateMeanAndDepthValues() {

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

bool OrbMapPoint::GetTrackInView()
{
    return m_trackInView;
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

void OrbMapPoint::SetTrackInView(bool trackInView)
{
    m_trackInView = trackInView;
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

} // namespace sensation
} // namespace logic
} // namespace opendlv