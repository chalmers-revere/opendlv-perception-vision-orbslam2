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

#ifndef ORBKEYFRAME_HPP
#define ORBKEYFRAME_HPP


#include <orbframe.hpp>
#include <orbkeyframedatabase.hpp>
#include <orbmap.hpp>
#include <orbmappoint.hpp>
#include <orbbowvector.hpp>
#include <orbfeaturevector.hpp>
#include <memory>
class OrbMap;
class OrbMapPoint;
class OrbKeyFrameDatabase;

class OrbKeyFrame : public std::enable_shared_from_this<OrbKeyFrame>
{
public:
    OrbKeyFrame(std::shared_ptr<OrbFrame> frame, std::shared_ptr<OrbMap> map,
                std::shared_ptr<OrbKeyFrameDatabase> keyFrameDatabase);

    // Pose functions
    void SetPose(const cv::Mat &cameraPose);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();

    static bool FrameIDCompare(std::shared_ptr<OrbKeyFrame> keyFrame1, std::shared_ptr<OrbKeyFrame> keyFrame2){
        return keyFrame1->m_id < keyFrame2->m_id;
    }

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(std::shared_ptr<OrbKeyFrame> keyFrame, const int &weight);
    void EraseConnection(std::shared_ptr<OrbKeyFrame> keyFrame);
    void UpdateConnections();
    void UpdateBestCovisibles();
    std::set<std::shared_ptr<OrbKeyFrame>> GetConnectedKeyFrames();
    std::vector<std::shared_ptr<OrbKeyFrame>> GetVectorCovisibleKeyFrames();
    std::vector<std::shared_ptr<OrbKeyFrame>> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<std::shared_ptr<OrbKeyFrame>> GetCovisiblesByWeight(const int &weight);
    int GetWeight(std::shared_ptr<OrbKeyFrame> keyFrame);

    // Spanning tree functions
    void AddChild(std::shared_ptr<OrbKeyFrame> keyFrame);
    void EraseChild(std::shared_ptr<OrbKeyFrame> keyFrame);
    void ChangeParent(std::shared_ptr<OrbKeyFrame> keyFrame);
    std::set<std::shared_ptr<OrbKeyFrame>> GetChilds();
    std::shared_ptr<OrbKeyFrame> GetParent();
    bool hasChild(std::shared_ptr<OrbKeyFrame> keyFrame);

    // Loop Edges
    void AddLoopEdge(std::shared_ptr<OrbKeyFrame> keyFrame);
    std::set<std::shared_ptr<OrbKeyFrame>> GetLoopEdges();

    // MapPoint observation functions
    void AddMapPoint(std::shared_ptr<OrbMapPoint> mapPoint, const size_t &idx);
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(std::shared_ptr<OrbMapPoint> mapPoint);
    void ReplaceMapPointMatch(const size_t &idx, std::shared_ptr<OrbMapPoint> mapPoint);
    std::set<std::shared_ptr<OrbMapPoint>> GetMapPoints();
    std::vector<std::shared_ptr<OrbMapPoint>> GetMapPointMatches();
    int TrackedMapPoints(const int &minimumObservations);
    std::shared_ptr<OrbMapPoint> GetMapPoint(const size_t &idx);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    cv::Mat UnprojectStereo(int i);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b)
    {
        return a>b;
    }

    static bool lId(std::shared_ptr<OrbKeyFrame> keyFrame1, std::shared_ptr<OrbKeyFrame> keyFrame2)
    {
        return keyFrame1->m_id < keyFrame2->m_id;
    }

    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:
    static long unsigned int nNextId;
    long unsigned int m_id = {};
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int m_loopQuery;
    int m_loopWords;
    float mLoopScore = {};
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore = {};

    // Variables used by loop closing
    cv::Mat mTcwGBA = {};
    cv::Mat mTcwBefGBA = {};
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth; // negative value for monocular points
    const cv::Mat mDescriptors;

    //BoW
    OrbBowVector m_bagOfWords;
    OrbFeatureVector m_features;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp = {};

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    const cv::Mat mK;

    // The following variables need to be accessed trough a mutex to be thread safe.
private:

    // SE3 Pose and camera center
    cv::Mat m_cameraPose = {}; // Tcw, Pose
    cv::Mat m_reverseCameraPose = {}; // Twc, Inverse pose
    cv::Mat m_cameraCenter = {}; // Ow, Camera Center

    cv::Mat Cw = {}; // Stereo middle point. Only for visualization

    // MapPoints associated to keypoints
    std::vector<std::shared_ptr<OrbMapPoint>> m_mapPoints;

    // BoW
    std::shared_ptr<OrbKeyFrameDatabase> m_keyFrameDatabase;
    std::shared_ptr<OrbVocabulary> m_orbVocabulary;

    // Grid over the image to speed up feature matching
    std::vector<std::vector<std::vector<size_t>>> m_grid = {};

    std::map<std::shared_ptr<OrbKeyFrame>,int> m_connectedKeyFrameWeights = {};
    std::vector<std::shared_ptr<OrbKeyFrame>> m_orderedConnectedKeyFrames = {};
    std::vector<int> m_orderedWeights = {};

    // Spanning Tree and Loop Edges
    bool m_isFirstConnection;
    std::shared_ptr<OrbKeyFrame> m_parent;
    std::set<std::shared_ptr<OrbKeyFrame>> m_children = {};
    std::set<std::shared_ptr<OrbKeyFrame>> m_loopEdges = {};

    // Bad flags
    bool m_shoulNotBeErased;
    bool m_shouldBeErased;
    bool m_isBad;

    float mHalfBaseline; // Only for visualization

    std::shared_ptr<OrbMap> m_map;

    std::mutex m_poseMutex = {};
    std::mutex m_connectionsMutex = {};
    std::mutex m_featuresMutex = {};
};


#endif //ORBKEYFRAME_HPP