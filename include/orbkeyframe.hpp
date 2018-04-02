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

#ifndef ORBKEYFRAME_HPP
#define ORBKEYFRAME_HPP


#include <orbframe.hpp>
#include <orbkeyframedatabase.hpp>
#include <orbmap.hpp>
#include <orbmappoint.hpp>

class OrbMap;
class OrbMapPoint;

class OrbKeyFrame
{
public:
    OrbKeyFrame(OrbFrame &F, std::shared_ptr<OrbMap> pMap, std::shared_ptr<OrbKeyFrameDatabase> pKFDB);

    // Pose functions
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(std::shared_ptr<OrbKeyFrame> pKF, const int &weight);
    void EraseConnection(std::shared_ptr<OrbKeyFrame> pKF);
    void UpdateConnections();
    void UpdateBestCovisibles();
    std::set<std::shared_ptr<OrbKeyFrame>> GetConnectedKeyFrames();
    std::vector<std::shared_ptr<OrbKeyFrame>> GetVectorCovisibleKeyFrames();
    std::vector<std::shared_ptr<OrbKeyFrame>> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<std::shared_ptr<OrbKeyFrame>> GetCovisiblesByWeight(const int &w);
    int GetWeight(std::shared_ptr<OrbKeyFrame> pKF);

    // Spanning tree functions
    void AddChild(std::shared_ptr<OrbKeyFrame> pKF);
    void EraseChild(std::shared_ptr<OrbKeyFrame> pKF);
    void ChangeParent(std::shared_ptr<OrbKeyFrame> pKF);
    std::set<std::shared_ptr<OrbKeyFrame>> GetChilds();
    std::shared_ptr<OrbKeyFrame> GetParent();
    bool hasChild(std::shared_ptr<OrbKeyFrame> pKF);

    // Loop Edges
    void AddLoopEdge(std::shared_ptr<OrbKeyFrame> pKF);
    std::set<std::shared_ptr<OrbKeyFrame>> GetLoopEdges();

    // MapPoint observation functions
    void AddMapPoint(std::shared_ptr<OrbMapPoint> pMP, const size_t &idx);
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(std::shared_ptr<OrbMapPoint> pMP);
    void ReplaceMapPointMatch(const size_t &idx, std::shared_ptr<OrbMapPoint> pMP);
    std::set<std::shared_ptr<OrbMapPoint>> GetMapPoints();
    std::vector<std::shared_ptr<OrbMapPoint>> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
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

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(std::shared_ptr<OrbKeyFrame> pKF1, std::shared_ptr<OrbKeyFrame> pKF2){
        return pKF1->mnId<pKF2->mnId;
    }


    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    static long unsigned int nNextId;
    long unsigned int mnId = {};
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
    long unsigned int mnLoopQuery;
    int mnLoopWords;
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
    OrbBowVector mBowVec;
    OrbFeatureVector mFeatVec;

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
protected:

    // SE3 Pose and camera center
    cv::Mat Tcw = {};
    cv::Mat Twc = {};
    cv::Mat Ow = {};

    cv::Mat Cw = {}; // Stereo middel point. Only for visualization

    // MapPoints associated to keypoints
    std::vector<std::shared_ptr<OrbMapPoint>> mvpMapPoints;

    // BoW
    std::shared_ptr<OrbKeyFrameDatabase> mpKeyFrameDB;
    std::shared_ptr<OrbVocabulary> mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid = {};

    std::map<std::shared_ptr<OrbKeyFrame>,int> mConnectedKeyFrameWeights = {};
    std::vector<std::shared_ptr<OrbKeyFrame>> mvpOrderedConnectedKeyFrames = {};
    std::vector<int> mvOrderedWeights = {};

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    std::shared_ptr<OrbKeyFrame> mpParent;
    std::set<std::shared_ptr<OrbKeyFrame>> mspChildrens = {};
    std::set<std::shared_ptr<OrbKeyFrame>> mspLoopEdges = {};

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;

    float mHalfBaseline; // Only for visualization

    std::shared_ptr<OrbMap> mpMap;

    std::mutex mMutexPose = {};
    std::mutex mMutexConnections = {};
    std::mutex mMutexFeatures = {};
};


#endif //ORBKEYFRAME_HPP