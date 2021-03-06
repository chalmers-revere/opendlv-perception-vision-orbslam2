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

#ifndef ORBMAPPOINT_HPP
#define ORBMAPPOINT_HPP

#include <map>
#include <memory>
#include <mutex>
#include <vector>
#include <orbframe.hpp>
#include <orbmap.hpp>
#include <orbconverter.hpp>
#include <orbkeyframe.hpp>
#include <array>

class OrbKeyFrame;
class OrbFrame;
class OrbMap;

class OrbMapPoint : public std::enable_shared_from_this<OrbMapPoint>
{
public:
    OrbMapPoint(const cv::Mat &position, std::shared_ptr<OrbKeyFrame> refenceKeyFrame, std::shared_ptr<OrbMap> map);
    OrbMapPoint(const cv::Mat &position, std::shared_ptr<OrbKeyFrame> refenceKeyFrame, std::shared_ptr<OrbMap> map, const int &keyPointIndex);
    OrbMapPoint(const cv::Mat &position, std::shared_ptr<OrbFrame> refenceKeyFrame, std::shared_ptr<OrbMap> map, const int &keyPointIndex);
    ~OrbMapPoint();
    int m_constructorTag = -1;
    std::map<std::shared_ptr<OrbKeyFrame>, size_t> GetObservingKeyframes();

    void SetWorldPosition(const cv::Mat &position);
    cv::Mat GetWorldPosition();

    cv::Mat GetMeanViewingDirection();
    std::shared_ptr<OrbKeyFrame> GetReferenceKeyFrame();

    int GetObservingKeyFrameCount();
    int GetSequenceId();
    void AddObservingKeyframe(std::shared_ptr<OrbKeyFrame> keyFrame,size_t idx);
    void EraseObservingKeyframe(std::shared_ptr<OrbKeyFrame> keyFrame);

    int GetObeservationIndexOfKeyFrame(std::shared_ptr<OrbKeyFrame> keyFrame);
    bool KeyFrameInObservingKeyFrames(std::shared_ptr<OrbKeyFrame> keyFrame);

    void SetCorruptFlag();
    bool IsCorrupt();

    void Replace(std::shared_ptr<OrbMapPoint> orbMapPoint);
    std::shared_ptr<OrbMapPoint> GetReplaced();

    void IncreaseVisible(int n = 1);
    void IncreaseFound(int n = 1);
    float GetFoundRatio();
    inline int GetFound() { return m_foundCounter; }
    float getTrackProjX() { return mTrackProjX; }
    float getTrackProjY() { return mTrackProjY; }
    float getTrackProjXR() { return mTrackProjXR; }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateMeanAndDepthValues();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, std::shared_ptr<OrbKeyFrame> keyFrame);
    int PredictScale(const float &currentDist, std::shared_ptr<OrbFrame> frame);

    int GetTrackScaleLevel();
    bool GetTrackInView();
    float GTrackViewCos();
    long unsigned int GetTrackReferenceForFrame();
    long unsigned int GetLastFrameSeen();
    long unsigned int GetBALocalForKF();
    long unsigned int GetFuseCandidateForKF();
    long unsigned int GetLoopPointForKF();
    long unsigned int GetCorrectedByKF();
    long unsigned int GetCorrectedReference();
    //cv::Mat GetPosGBA();
    //long unsigned int GetBAGlobalForKF();
    long int GetFirstKeyFrameId(){ return m_firstKeyframeId;};

    //void SetTrackScaleLevel(long unsigned int TrackScaleLevel);
    void SetTrackInView(bool TrackInView);
    //void SetackViewCos(long unsigned int ackViewCos);
    void SetTrackReferenceForFrame(long unsigned int TrackReferenceForFrame);
    void SetLastFrameSeen(long unsigned int LastFrameSeen);
    void SetBALocalForKF(long unsigned int BALocalForKF);
    void SetFuseCandidateForKF(long unsigned int FuseCandidateForKF);
    void SetLoopPointForKF(long unsigned int LoopPointForKF);
    void SetCorrectedByKF(long unsigned int CorrectedByKF);
    void SetCorrectedReference(long unsigned int CorrectedReference);
    //void SetPosGBA(long unsigned int PosGBA);
    //void SetBAGlobalForKF(long unsigned int BAGlobalForKF);
    void SetTrackProjX(const float d);
    void SetTrackProjXR(float d);
    void SetTrackProjY(const float d);
    void SetnTrackScaleLevel(const int i);
    void SetTrackViewCos(const float d);

    static std::mutex mGlobalMutex;
    static long unsigned int nNextId;       //Static because ID is iterated for each new mappoint so needs to be accesible by all
    cv::Mat mPosGBA = {};
    long unsigned int mnBAGlobalForKF = {};

private:
    long unsigned int m_sequenceId = 0;
    long int m_firstKeyframeId = {};
    long int m_FirstKeyFrame = {};
    long unsigned int m_observingKeyFramesCount = {};

    // Variables used by the tracking
    float mTrackProjX = {};
    float mTrackProjY = {};
    float mTrackProjXR = {};
    bool mbTrackInView = {};
    int mnTrackScaleLevel = {};
    float mTrackViewCos = {};
    long unsigned int mnTrackReferenceForFrame = {};
    long unsigned int mnLastFrameSeen = {};

    // Variables used by local mapping
    long unsigned int mnBALocalForKF = {};
    long unsigned int mnFuseCandidateForKF = {};

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF = {};
    long unsigned int mnCorrectedByKF = {};
    long unsigned int mnCorrectedReference = {};

    // mutexed below

    // Position in absolute coordinates
    cv::Mat m_worldPosition = {};

     // Keyframes observing the point and associated index in keyframe
     std::map<std::shared_ptr<OrbKeyFrame>,size_t> m_observingKeyframes = {};

    // Mean viewing direction
    cv::Mat m_meanViewingDirection = {};

    // Best descriptor to fast matching
    cv::Mat m_descriptor = {};

     // Reference KeyFrame
     std::shared_ptr<OrbKeyFrame> m_refenceKeyFrame = {};

    // Tracking counters
    int m_visibleCounter = 1;
    int m_foundCounter = 1;

    // Bad flag (we do not currently erase MapPoint from memory)
    bool m_corrupt = false;
    std::shared_ptr<OrbMapPoint> m_replaced = {};

    // Scale invariance distances
    float m_minDistance = {};
    float m_maxDistance = {};

    std::shared_ptr<OrbMap> m_map = {};

    std::mutex m_constructorMutex = {};
    std::mutex m_positionMutex = {};
    std::mutex m_featureMutex = {};
};

#endif // ORBMAPPOINT_HPP