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

#ifndef TRACKING_HPP
#define TRACKING_HPP

#include <memory>
#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "orbkeyframedatabase.hpp"
#include "mapping.hpp"
#include "loopclosing.hpp"
#include "orbinitializer.hpp"
#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
#include <opencv2/core/mat.hpp>

class LoopClosing;
class Selflocalization;
class Mapping;
class Tracking
{

public:
    Tracking(std::shared_ptr<Selflocalization> selfLocalization, std::shared_ptr<OrbVocabulary> pVoc, std::shared_ptr<OrbMap> pMap,
             std::shared_ptr<OrbKeyFrameDatabase> pKFDB, std::map<std::string, std::string> commandlineArgs, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(std::shared_ptr<Mapping> pLocalMapper);
    void SetLoopClosing(std::shared_ptr<LoopClosing> pLoopClosing);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const std::string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);
    bool GetTrackingState(){ return m_trackingState == OK;};

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    // Input sensor
    int mSensor;

    // Current Frame
    std::shared_ptr<OrbFrame> mCurrentFrame = {};
    cv::Mat mImGray = {};

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches = {};
    std::vector<int> mvIniMatches = {};
    std::vector<cv::Point2f> mvbPrevMatched = {};
    std::vector<cv::Point3f> mvIniP3D = {};
    std::shared_ptr<OrbFrame> mInitialFrame = {};

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    std::list<cv::Mat> mlRelativeFramePoses = {};
    std::list<std::shared_ptr<OrbKeyFrame>> mlpReferences = {};
    std::list<double> mlFrameTimes = {};
    std::list<bool> mlbLost = {};

    void Reset();

private:

    // Main tracking function. It is independent of the input sensor.
    void Track();
    bool InitalizeTracking();
    void Calibrate(std::map<std::string, std::string> commandlineArgs);
    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    // True if local mapping is deactivated and we are performing only localization
    bool m_onlyTracking;
    eTrackingState m_trackingState;
    eTrackingState mLastProcessedState = {};

    //Other Thread Pointers
    std::shared_ptr<Mapping> mpLocalMapper = {};
    std::shared_ptr<LoopClosing> mpLoopClosing = {};

    //ORB
    std::shared_ptr<OrbExtractor> mpORBextractorLeft = {}, mpORBextractorRight = {};
    std::shared_ptr<OrbExtractor> mpIniORBextractor = {};

    //BoW
    std::shared_ptr<OrbVocabulary> mpORBVocabulary;
    std::shared_ptr<OrbKeyFrameDatabase> mpKeyFrameDB;

    // Initalization (only for monocular)
    std::shared_ptr<OrbInitializer> mpInitializer;

    //Local Map
    std::shared_ptr<OrbKeyFrame> mpReferenceKF = {};
    std::vector<std::shared_ptr<OrbKeyFrame>> mvpLocalKeyFrames = {};
    std::vector<std::shared_ptr<OrbMapPoint>> mvpLocalMapPoints = {};

    // System
    std::shared_ptr<Selflocalization> mpSystem;

    //Map
    std::shared_ptr<OrbMap> mpMap;

    //Calibration matrix
    cv::Mat mK = {};
    cv::Mat mDistCoef = {};
    float mbf = {};

    //New KeyFrame rules (according to fps)
    int mMinFrames = {};
    int mMaxFrames = {};

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth = {};

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor = {};

    //Current matches in frame
    int mnMatchesInliers = {};

    //Last Frame, KeyFrame and Relocalisation Info
    std::shared_ptr<OrbKeyFrame> mpLastKeyFrame = {};
    std::shared_ptr<OrbFrame> mLastFrame = {};
    unsigned int mnLastKeyFrameId = {};
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity = {};

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB = {};

    std::list<std::shared_ptr<OrbMapPoint>> mlpTemporalPoints = {};
};


#endif
