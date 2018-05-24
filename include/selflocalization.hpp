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

#ifndef SELFLOCALIZATION_HPP
#define SELFLOCALIZATION_HPP

#include <thread>
#include <iomanip>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "WGS84toCartesian.hpp"
#include "tracking.hpp"
#include "mapping.hpp"
#include "loopclosing.hpp"
#include "orbextractor.hpp"
#include "orbvocabulary.hpp"
#include "orbmap.hpp"
#include "kittirunner.hpp"


class Selflocalization
{
public:

    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };

    Selflocalization(std::map<std::string, std::string> commandlineArgs);
    Selflocalization(Selflocalization const &) = delete;
    Selflocalization &operator=(Selflocalization const &) = delete;
    ~Selflocalization();
    void nextContainer(cv::Mat &img);
    void runKitti(std::string kittiPath);
    std::pair<bool,opendlv::logic::sensation::Geolocation> sendPose();
    void sendMap(size_t &lastMapPoint,uint32_t &lastSentCameraIndex, uint32_t &lastSentIndex, size_t i, cluon::OD4Session &od4);
    void writeToMapFile(std::string filepath);
    void writeToFpsFile(std::string filepath, std::vector<long unsigned int> mapSizeVector, std::vector<double> fpsVector);
    void Shutdown();
    void Track(cv::Mat &imLeft, cv::Mat &imRight, double &timestamp);
    void Track(cv::Mat &imLeft, double &timestamp);


    // Reset the system (clear map)
    void Reset();
    opendlv::proxy::OrbslamMap sendToWebb();

private:
    void setUp(std::map<std::string, std::string> commandlineArgs);
    void setUpRealtime(std::map<std::string, std::string> commandlineArgs);
    void tearDown();
    opendlv::proxy::PointCloudReading CreatePointCloudFromMap();
    bool m_isMonocular;
    int m_saveCounter = 0;
    std::shared_ptr<Mapping> m_pMapper;
    std::shared_ptr<Tracking> m_pTracker;
    std::shared_ptr<LoopClosing> m_pLoopCloser;
    std::shared_ptr<std::thread> m_pMappingThread;
    std::shared_ptr<std::thread> m_pLoopClosingThread;

    std::shared_ptr<OrbExtractor> m_pExtractOrb;
    std::shared_ptr<OrbVocabulary> m_pVocabulary;
    std::shared_ptr<OrbKeyFrameDatabase> m_pKeyFrameDatabase;
    std::shared_ptr<OrbMap> m_map;

    std::mutex mMutexReset = {};
    bool m_reset = false;
    std::chrono::steady_clock::time_point m_last_envelope_ts;

    std::array<double,2> m_gpsReference = {};
    double m_referenceHeading=0;
    int m_cid = 0;

    const double DEG2RAD = 0.017453292522222; // PI/180.0
    const double RAD2DEG = 57.295779513082325; // 1.0 / DEG2RAD;
    const double PI = 3.14159265f;
    //Rectification parameters
    cv::Mat rmap[2][2];
    float m_resizeScale;
};


#endif
