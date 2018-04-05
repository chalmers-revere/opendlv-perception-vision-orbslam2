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

#ifndef ORBOPTIMIZER_HPP
#define ORBOPTIMIZER_HPP



#include "orbmap.hpp"
#include "orbmappoint.hpp"
#include "orbkeyframe.hpp" 
//DENNA SKA MÄÄ #include "orbloopclosing.hpp"
#include "loopclosing.hpp"
#include "orbframe.hpp"

#include "g2o/types/sim3/types_seven_dof_expmap.h"

class LoopClosing;
class OrbOptimizer {
 public:
  OrbOptimizer();
  ~OrbOptimizer();

    void static GlobalBundleAdjustemnt(std::shared_ptr<OrbMap> pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);
    void static BundleAdjustment(const std::vector<std::shared_ptr<OrbKeyFrame>> &vpKF, const std::vector<std::shared_ptr<OrbMapPoint>> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);
    int static PoseOptimization(std::shared_ptr<OrbFrame> pFrame);
    void static LocalBundleAdjustment(std::shared_ptr<OrbKeyFrame> keyframe, bool *pbStopFlag, std::shared_ptr<OrbMap> pMap);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    void static OptimizeEssentialGraph(std::shared_ptr<OrbMap> pMap, std::shared_ptr<OrbKeyFrame> pLoopKF, std::shared_ptr<OrbKeyFrame> pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const std::map<std::shared_ptr<OrbKeyFrame>, std::set<std::shared_ptr<OrbKeyFrame>> > &LoopConnections,
                                       const bool &bFixScale);

    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    static int OptimizeSim3(std::shared_ptr<OrbKeyFrame> keyFrame1, std::shared_ptr<OrbKeyFrame> keyFrame2, std::vector<std::shared_ptr<OrbMapPoint>> &vpMatches1, g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);
	
};


#endif //ORBMAP_HPP