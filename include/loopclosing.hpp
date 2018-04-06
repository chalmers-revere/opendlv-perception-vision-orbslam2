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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "orbkeyframe.hpp"
#include "mapping.hpp"
#include "orbmap.hpp"
#include "orbvocabulary.hpp"
#include "tracking.hpp"

#include "orbkeyframedatabase.hpp"

#include <thread>
#include <mutex>
#include "g2o/types/sim3/types_seven_dof_expmap.h"

class Tracking;
class Mapping;
class KeyFrameDatabase;

class LoopClosing
{
public:
    void RequestReset();

    typedef std::pair<std::set<std::shared_ptr<OrbKeyFrame>>,int> ConsistentGroup;    
    typedef std::map<std::shared_ptr<OrbKeyFrame>,g2o::Sim3,std::less<std::shared_ptr<OrbKeyFrame>>,
        Eigen::aligned_allocator<std::pair<const std::shared_ptr<OrbKeyFrame>, g2o::Sim3> > > KeyFrameAndPose;

public:

    LoopClosing(std::shared_ptr<OrbMap> pMap, std::shared_ptr<OrbKeyFrameDatabase> pDB, std::shared_ptr<OrbVocabulary> pVoc,const bool bFixScale);

    void SetTracker(std::shared_ptr<Tracking> pTracker);

    void SetLocalMapper(std::shared_ptr<Mapping> pLocalMapper);

    // Main function
    void Run();

    void InsertKeyFrame(std::shared_ptr<OrbKeyFrame> pKF);

    // This function will run in a separate thread
    void RunGlobalBundleAdjustment(unsigned long nLoopKF);

    bool isRunningGBA(){
        std::unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    bool isFinishedGBA(){
        std::unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }   

    void RequestFinish();

    bool isFinished();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

    bool CheckNewKeyFrames();

    bool DetectLoop();

    bool ComputeSim3();

    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);

    void CorrectLoop();

    void ResetIfRequested();
    bool mbResetRequested = {};
    std::mutex mMutexReset = {};

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested = {};
    bool mbFinished = {};
    std::mutex mMutexFinish = {};

    std::shared_ptr<OrbMap> mpMap;
    std::shared_ptr<Tracking> mpTracker = {};

    std::shared_ptr<OrbKeyFrameDatabase> mpKeyFrameDB;
    std::shared_ptr<OrbVocabulary> mpORBVocabulary;

    std::shared_ptr<Mapping> mpLocalMapper = {};

    std::list<std::shared_ptr<OrbKeyFrame>> mlpLoopKeyFrameQueue = {};

    std::mutex mMutexLoopQueue = {};

    // Loop detector parameters
    float mnCovisibilityConsistencyTh = {};

    // Loop detector variables
    std::shared_ptr<OrbKeyFrame> mpCurrentKF = {};
    std::shared_ptr<OrbKeyFrame> mpMatchedKF = {};
    std::vector<ConsistentGroup> mvConsistentGroups = {};
    std::vector<std::shared_ptr<OrbKeyFrame>> mvpEnoughConsistentCandidates = {};
    std::vector<std::shared_ptr<OrbKeyFrame>> mvpCurrentConnectedKFs = {};
    std::vector<std::shared_ptr<OrbMapPoint>> mvpCurrentMatchedPoints = {};
    std::vector<std::shared_ptr<OrbMapPoint>> mvpLoopMapPoints = {};
    cv::Mat mScw = {};
    g2o::Sim3 mg2oScw = {};

    long unsigned int mLastLoopKFid = {};

    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA = {};
    bool mbFinishedGBA = {};
    bool mbStopGBA = {};
    std::mutex mMutexGBA = {};
    std::shared_ptr<std::thread> mpThreadGBA = {};

    // Fix scale in the stereo/RGB-D case
    bool mbFixScale = {};


    bool mnFullBAIdx = {};
};

#endif // LOOPCLOSING_H
