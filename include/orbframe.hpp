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

#ifndef LOGIC_SENSATION_SELFLOCALIZATION_ORBFRAME_HPP
#define LOGIC_SENSATION_SELFLOCALIZATION_ORBFRAME_HPP

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "orbvocabulary.hpp"
#include <memory>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <vector>
#include <list>
#include <utility>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <orbmappoint.hpp>
#include <orbextractor.hpp>
#include <orbkeyframe.hpp>
#include <orbbowvector.hpp>
#include <orbfeaturevector.hpp>

class OrbMapPoint;

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class OrbMapPoint;
class OrbKeyFrame;

class OrbFrame : public std::enable_shared_from_this<OrbFrame>
{
public:
    OrbFrame(const std::shared_ptr<OrbFrame>&frame);

    // Constructor for stereo cameras.
    OrbFrame(const cv::Mat &leftImage, const cv::Mat &rightImage, const double &timeStamp,
             std::shared_ptr<OrbExtractor> leftExtractor, std::shared_ptr<OrbExtractor> rightExtractor,
             std::shared_ptr<OrbVocabulary> orbVocabulary, cv::Mat &calibrationMatrix, cv::Mat &distanceCoefficient,
             const float &stereoBaseline, const float &depthThreshold,std::array<float, 4> boundingBox);

    // Constructor for RGB-D cameras.
    OrbFrame(const cv::Mat &greyImage, const cv::Mat &imageDepth, const double &timeStamp,
             std::shared_ptr<OrbExtractor> extractor, std::shared_ptr<OrbVocabulary> orbVocabulary,
             cv::Mat &calibrationMatrix, cv::Mat &distanceCoefficient, const float &stereoBaseline,
             const float &depthThreshold);

    // Constructor for Monocular cameras.
    OrbFrame(const cv::Mat &greyImage, const double &timeStamp, std::shared_ptr<OrbExtractor> extractor,
             std::shared_ptr<OrbVocabulary> orbVocabulary, cv::Mat &calibrationMatrix, cv::Mat &distanceCoefficient,
             const float &stereoBaseline, const float &depthThreshold,std::array<float, 4> boundingBox);


    // Extract ORB on the image. 0 for left image and 1 for right image.
    void ExtractORB(int rightImage, const cv::Mat &image);

    // Compute Bag of Words representation.
    void ComputeBoW();

    // Set the camera pose.
    void SetPose(cv::Mat cameraPose);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter()
    {
        return m_cameraCenter.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse()
    {
        return m_reverseRotation.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool IsInFrustum(std::shared_ptr<OrbMapPoint> mapPoint, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &keyPoint, int &xPosition, int &yPosition);

    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    void ComputeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imageDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    cv::Mat UnprojectStereo(const int &i);

public:
    // Vocabulary used for relocalization.
    std::shared_ptr<OrbVocabulary> m_ORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    std::shared_ptr<OrbExtractor> m_ORBextractorLeft, m_ORBextractorRight;

    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    // Stereo baseline multiplied by fx.
    float mbf;

    // Stereo baseline in meters.
    float mb = {};

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;

    // Number of KeyPoints.
    int N = {};

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> m_keys = {}, m_keysRight = {};
    std::vector<cv::KeyPoint> m_undistortedKeys = {};

    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight = {};
    std::vector<float> m_depths = {};

    // Bag of Words Vector structures.
    OrbBowVector mBowVec = {};
    OrbFeatureVector mFeatVec = {};

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat m_descriptors = {}, m_descriptorsRight = {};

    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<std::shared_ptr<OrbMapPoint>> m_mapPoints = {};

    // Flag to identify outlier associations.
    std::vector<bool> m_outliers = {};

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float m_gridElementWidthInverse;
    static float m_gridElementHeightInverse;
    std::vector<std::size_t> m_grid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    cv::Mat mTcw = {};

    // Current and Next Frame id.
    static long unsigned int m_nextId;
    long unsigned int mnId = {};

    // Reference Keyframe.
    std::shared_ptr<OrbKeyFrame> m_referenceKeyFrame = {};

    // Scale pyramid info.
    int m_scaleLevels = {};
    float m_scaleFactor = {};
    float m_logScaleFactor = {};
    std::vector<float> m_scaleFactors = {};
    std::vector<float> m_invScaleFactors = {};
    std::vector<float> m_levelSigma2 = {};
    std::vector<float> m_inverseLevelSigma2 = {};

    // Undistorted Image Bounds (computed once).
    static float m_minX;
    static float m_maxX;
    static float m_minY;
    static float m_maxY;

    static bool m_initialComputations;

private:
    void CommonSetup();
    void InitialComputation(cv::Mat calibrationMatrix, cv::Mat image);

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    //Removes the keypoints inside the bounding box in both images
    void FilterKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &leftImage);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    std::array<float, 4> m_boundingBox = {};
    // Rotation, translation and camera center
    cv::Mat m_rotation = {}; //Rcw, rotation.
    cv::Mat m_reversePose = {}; //Tcw, inverse pose.
    cv::Mat m_reverseRotation = {}; // Rwc, reverse rotation.
    cv::Mat m_cameraCenter = {}; //Ow camera center==mtwc
};

#endif
