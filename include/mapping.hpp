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

#ifndef MAPPING_HPP
#define MAPPING_HPP

//#include "cluon-complete.hpp"
//#include "opendlv-standard-message-set.hpp"

#include <memory>

#include <iostream>
#include <memory>
#include <vector>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "orbkeyframe.hpp"
#include "orbmap.hpp"
#include "loopclosing.hpp"
//include tracking
#include "orbkeyframedatabase.hpp"

class LoopClosing;
class OrbMap;

class Mapping {
 public:
  Mapping(std::shared_ptr<OrbMap> pMap, const bool bMonocular);
	//void setTracker(std::shared_ptr<Tracking> pTracker);
	void SetLoopCloser(std::shared_ptr<LoopClosing> pLoopCloser);
	void Run();


	void InsertKeyFrame(std::shared_ptr<OrbKeyFrame> pKF);
	
  	void RequestStop();
  	void RequestReset();
  	bool Stop();
  	void Release();
  	bool isStopped();
  	bool stopRequested();
  	bool AcceptKeyFrames();
  	void SetAcceptKeyFrames(bool flag);
  	bool SetNotStop(bool flag);
  	void InterruptBA();
  	void RequestFinish();
  	bool isFinished();

  	int KeyframesInQueue(){
     std::unique_lock<std::mutex> lock(mMutexNewKFs);
     return mlNewKeyFrames.size();
  }
 private:

  bool CheckNewKeyFrames();

  void ProcessNewKeyFrame();
  void CreateNewMapPoints();
  void MapPointCulling();
  void SearchInNeighbors();
  void KeyFrameCulling();


	cv::Mat ComputeF12(std::shared_ptr<OrbKeyFrame> &pKF1, std::shared_ptr<OrbKeyFrame> &pKF2);
	cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

  void ResetIfRequested();
  bool CheckFinish();
  void SetFinish();

  std::mutex mMutexNewKFs = {};
  std::mutex mMutexStop = {};
  std::mutex mMutexFinish = {};
  std::mutex mMutexAccept = {};
  std::mutex mMutexReset = {};
  bool mbMonocular;
  bool mbResetRequested;
  bool mbFinishRequested;
  bool mbFinished;

   std::shared_ptr<OrbMap> mpMap;
   std::shared_ptr<LoopClosing> mpLoopCloser = {};

  bool mbAbortBA;
  bool mbStopped;
  bool mbStopRequested;
  bool mbNotStop;
  bool mbAcceptKeyFrames;

  std::list<std::shared_ptr<OrbKeyFrame>> mlNewKeyFrames = {};
  std::shared_ptr<OrbKeyFrame> mpCurrentKeyFrame = {};
  std::list<std::shared_ptr<OrbMapPoint>> mlpRecentAddedMapPoints = {};
	//std::shared_ptr<Map> m_pMap;
	//std::shared_ptr<Tracking> m_pTracker;
	//std::shared_ptr<LoopClosing> m_pLoopCloser;
	/*Variables needed to initialize threads and databases*/
  /*
	
	std::shared_ptr<OrbVocabulary> m_pVocabulary;
	std::shared_ptr<Map> m_pMap;
	std::shared_ptr<KeyFrameDatabase> m_pKeyFrameDatabase;
	std::shared_ptr<Tracking> m_pTracker;
	std::shared_ptr<Mapping> m_pMapper;
	std::shared_ptr<LoopClosing> m_pLoopCloser;

	std::shared_ptr<std::thread> m_pMappingThread;
	std::shared_ptr<std::thread> m_pLoopClosingThread;


*/

	
};


#endif
