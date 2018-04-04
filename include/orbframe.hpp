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
#include <orbkeypoint.hpp>
#include <orbmappoint.hpp>
#include <orbextractor.hpp>
#include <orbkeyframe.hpp>

class OrbMapPoint;

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class OrbMapPoint;
class OrbKeyFrame;

class OrbFrame
{
public:
    OrbFrame(const std::shared_ptr<OrbFrame>&frame);

    // Constructor for stereo cameras.
    OrbFrame(const cv::Mat &leftImage, const cv::Mat &rightImage, const double &timeStamp,
             std::shared_ptr<OrbExtractor> leftExtractor, std::shared_ptr<OrbExtractor> rightExtractor,
             std::shared_ptr<OrbVocabulary> orbVocabulary, cv::Mat &calibrationMatrix, cv::Mat &distanceCoefficient,
             const float &stereoBaseline, const float &depthThreshold);

    // Constructor for RGB-D cameras.
    OrbFrame(const cv::Mat &greyImage, const cv::Mat &imageDepth, const double &timeStamp,
             std::shared_ptr<OrbExtractor> extractor, std::shared_ptr<OrbVocabulary> orbVocabulary,
             cv::Mat &calibrationMatrix, cv::Mat &distanceCoefficient, const float &stereoBaseline,
             const float &depthThreshold);

    // Constructor for Monocular cameras.
    OrbFrame(const cv::Mat &greyImage, const double &timeStamp, std::shared_ptr<OrbExtractor> extractor,
             std::shared_ptr<OrbVocabulary> orbVocabulary, cv::Mat &calibrationMatrix, cv::Mat &distanceCoefficient,
             const float &stereoBaseline, const float &depthThreshold);


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
    bool isInFrustum(std::shared_ptr<OrbMapPoint> mapPoint, float viewingCosLimit);

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
    std::shared_ptr<OrbVocabulary> mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    std::shared_ptr<OrbExtractor> mpORBextractorLeft, mpORBextractorRight;

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
    std::vector<cv::KeyPoint> mvKeys = {}, mvKeysRight = {};
    std::vector<cv::KeyPoint> mvKeysUn = {};

    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight = {};
    std::vector<float> mvDepth = {};

    // Bag of Words Vector structures.
    OrbBowVector mBowVec = {};
    OrbFeatureVector mFeatVec = {};

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors = {}, mDescriptorsRight = {};

    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<std::shared_ptr<OrbMapPoint>> mvpMapPoints = {};

    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier = {};

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    cv::Mat mTcw = {};

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId = {};

    // Reference Keyframe.
    std::shared_ptr<OrbKeyFrame> mpReferenceKF = {};

    // Scale pyramid info.
    int mnScaleLevels = {};
    float mfScaleFactor = {};
    float mfLogScaleFactor = {};
    std::vector<float> mvScaleFactors = {};
    std::vector<float> mvInvScaleFactors = {};
    std::vector<float> mvLevelSigma2 = {};
    std::vector<float> mvInvLevelSigma2 = {};

    // Undistorted Image Bounds (computed once).
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;


private:
    void CommonSetup();
    void InitialComputation(cv::Mat calibrationMatrix, cv::Mat image);

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &leftImage);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center
    cv::Mat m_rotation = {}; //Rcw, rotation.
    cv::Mat m_reversePose = {}; //Tcw, inverse pose.
    cv::Mat m_reverseRotation = {}; // Rwc, reverse rotation.
    cv::Mat m_cameraCenter = {}; //Ow camera center==mtwc
};

#endif