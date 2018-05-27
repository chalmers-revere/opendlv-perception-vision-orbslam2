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
    void WriteToPoseFile(const std::string &filename);
    void GetTrajectory(std::vector< std::pair<cv::Mat, cv::Mat>> & trajectory);
    void GetTrajectory(std::ofstream &stream);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const std::string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);
    bool GetTrackingState(){ return m_trackingState == OK;};

    // Tracking states
    enum trackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    // Input sensor
    int m_sensor;

    // Current Frame
    std::shared_ptr<OrbFrame> m_currentFrame = {};
    cv::Mat m_imageGray = {};

    // Initialization Variables (Monocular)
    std::vector<int> m_initialLastMatches = {};
    std::vector<int> m_initialMatches = {};
    std::vector<cv::Point2f> m_previouslyMatchedPoints = {};
    std::vector<cv::Point3f> m_initialPoints3D = {};
    std::shared_ptr<OrbFrame> m_initialFrame = {};

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    std::list<cv::Mat> m_relativeFramePoses = {};
    std::list<std::shared_ptr<OrbKeyFrame>> m_ReferenceKeyFrames = {};
    std::list<double> m_frameTimeStamps = {};
    std::list<bool> m_Lost = {};

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

    bool UpdatePoses();
    void ProcessKeyFrames();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool m_VO;

    // True if local mapping is deactivated and we are performing only localization
    bool m_onlyTracking;
    trackingState m_trackingState;
    trackingState m_lastProcessedState = {};

    //Other Thread Pointers
    std::shared_ptr<Mapping> m_localMapper = {};
    std::shared_ptr<LoopClosing> m_loopClosing = {};

    //ORB
    std::shared_ptr<OrbExtractor> m_ORBextractorLeft = {}, m_ORBextractorRight = {};
    std::shared_ptr<OrbExtractor> m_initialORBextractor = {};

    //BoW
    std::shared_ptr<OrbVocabulary> m_ORBVocabulary;
    std::shared_ptr<OrbKeyFrameDatabase> m_keyFrameDataBase;

    // Initalization (only for monocular)
    std::shared_ptr<OrbInitializer> m_initializer;

    //Local Map
    std::shared_ptr<OrbKeyFrame> m_referenceKeyFrame = {};
    std::vector<std::shared_ptr<OrbKeyFrame>> m_localKeyFrames = {};
    std::vector<std::shared_ptr<OrbMapPoint>> m_localMapPoints = {};

    // System
    std::shared_ptr<Selflocalization> m_selflocalization;

    //Map
    std::shared_ptr<OrbMap> m_orbMap;

    //Calibration matrix
    cv::Mat mK = {};
    cv::Mat m_distanceCoeffecient = {};
    float m_bf = {};

    //New KeyFrame rules (according to fps)
    int m_minFrames = {};
    int m_maxFrames = {};

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float m_thDepth = {};

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float m_depthMapFactor = {};

    //Current matches in frame
    int m_matchesInliers = {};

    //Box within which keyframes are removed
    std::array<float, 4> m_boundingBox = {};

    //Last Frame, KeyFrame and Relocalisation Info
    std::shared_ptr<OrbKeyFrame> m_lastKeyFrame = {};
    std::shared_ptr<OrbFrame> m_lastFrame = {};
    unsigned int m_lastKeyFrameId = {};
    unsigned int m_lastRelocationFrameId;

    //Motion Model
    cv::Mat m_velocity = {};

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool m_RGB = {};

    std::list<std::shared_ptr<OrbMapPoint>> m_temporalPoints = {};
};


#endif
