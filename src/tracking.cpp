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

#include <cmath>
#include <tracking.hpp>
#include <selflocalization.hpp>
#include <orbmatcher.hpp>
#include <orboptimizer.hpp>
#include <pnpsolver.hpp>

Tracking::Tracking(std::shared_ptr<Selflocalization> selfLocalization, std::shared_ptr<OrbVocabulary> pVoc,
                   std::shared_ptr<OrbMap> pMap, std::shared_ptr<OrbKeyFrameDatabase> pKFDB,
                   std::map<std::string, std::string> commandlineArgs, const int sensor):
            m_sensor(sensor), m_VO(false), m_onlyTracking(false), m_trackingState(NO_IMAGES_YET), m_ORBVocabulary(pVoc),
            m_keyFrameDataBase(pKFDB), m_initializer(static_cast<std::shared_ptr<OrbInitializer>>(NULL)), m_selflocalization(selfLocalization),
            m_orbMap(pMap), m_lastRelocationFrameId(0)
{
    // Load camera parameters from settings file
    //cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    this->Calibrate(commandlineArgs);

}

void Tracking::Calibrate(std::map<std::string, std::string> commandlineArgs) {
    float fx = std::stof(commandlineArgs["Camera.fx"]);
    float fy = std::stof(commandlineArgs["Camera.fy"]);
    float cx = std::stof(commandlineArgs["Camera.cx"]);
    float cy = std::stof(commandlineArgs["Camera.cy"]);

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = std::stof(commandlineArgs["Camera.k1"]);
    DistCoef.at<float>(1) = std::stof(commandlineArgs["Camera.k2"]);
    DistCoef.at<float>(2) = std::stof(commandlineArgs["Camera.p1"]);
    DistCoef.at<float>(3) = std::stof(commandlineArgs["Camera.p2"]);
    const float k3 = std::stof(commandlineArgs["Camera.k3"]);
    if(std::abs(k3)>0.0001f)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(m_distanceCoeffecient);

    m_bf = std::stof(commandlineArgs["Camera.bf"]);
    std::cout << "- bf: " << m_bf << std::endl;

    float fps = std::stof(commandlineArgs["Camera.fps"]);
    if(std::abs(fps-0)<0.0001f)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    m_minFrames = 0;
    m_maxFrames = static_cast<int>(fps);

    std::cout << std::endl << "Camera Parameters: " << std::endl;
    std::cout << "- fx: " << fx << std::endl;
    std::cout << "- fy: " << fy << std::endl;
    std::cout << "- cx: " << cx << std::endl;
    std::cout << "- cy: " << cy << std::endl;
    std::cout << "- k1: " << DistCoef.at<float>(0) << std::endl;
    std::cout << "- k2: " << DistCoef.at<float>(1) << std::endl;
    if(DistCoef.rows==5)
        std::cout << "- k3: " << DistCoef.at<float>(4) << std::endl;
    std::cout << "- p1: " << DistCoef.at<float>(2) << std::endl;
    std::cout << "- p2: " << DistCoef.at<float>(3) << std::endl;
    std::cout << "- fps: " << fps << std::endl;
    int nRGB = std::stoi(commandlineArgs["Camera.RGB"]);
    m_RGB = static_cast<bool>(nRGB);

    if(m_RGB)
        std::cout << "- color order: RGB (ignored if grayscale)" << std::endl;
    else
        std::cout << "- color order: BGR (ignored if grayscale)" << std::endl;

    // Load ORB parameters

    int nFeatures = std::stoi(commandlineArgs["ORBextractor.nFeatures"]);
    float fScaleFactor = std::stof(commandlineArgs["ORBextractor.scaleFactor"]);
    int nLevels = std::stoi(commandlineArgs["ORBextractor.nLevels"]);
    int fIniThFAST = std::stoi(commandlineArgs["ORBextractor.iniThFAST"]);
    int fMinThFAST = std::stoi(commandlineArgs["ORBextractor.minThFAST"]);

    //Read in bounding box in which keypoints are ignored
    if(commandlineArgs.count("BoundingBox.MaxX")>0){
        std::array<float, 4> boundingBox;
        boundingBox[0] = std::stoi(commandlineArgs["BoundingBox.MinX"]);
        boundingBox[1] = std::stoi(commandlineArgs["BoundingBox.MaxX"]);
        boundingBox[2] = std::stoi(commandlineArgs["BoundingBox.MinY"]);
        boundingBox[3] = std::stoi(commandlineArgs["BoundingBox.MaxY"]);
        std::cout << "BoundingBox: " << boundingBox[0] << boundingBox[1] << boundingBox[2] << boundingBox[3] << std::endl;
        m_boundingBox = boundingBox;
    }

    m_ORBextractorLeft = std::shared_ptr<OrbExtractor>(new OrbExtractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST));

    if(this->m_sensor==Selflocalization::STEREO)
        m_ORBextractorRight = std::shared_ptr<OrbExtractor>(new OrbExtractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST));

    if(this->m_sensor==Selflocalization::MONOCULAR)
        m_initialORBextractor = std::shared_ptr<OrbExtractor>(new OrbExtractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST));

    std::cout << std::endl  << "ORB Extractor Parameters: " << std::endl;
    std::cout << "- Number of Features: " << nFeatures << std::endl;
    std::cout << "- Scale Levels: " << nLevels << std::endl;
    std::cout << "- Scale Factor: " << fScaleFactor << std::endl;
    std::cout << "- Initial Fast Threshold: " << fIniThFAST << std::endl;
    std::cout << "- Minimum Fast Threshold: " << fMinThFAST << std::endl;

    if(this->m_sensor==Selflocalization::STEREO || this->m_sensor==Selflocalization::RGBD)
    {
        m_thDepth = m_bf*std::stof(commandlineArgs["ThDepth"])/fx;
        std::cout << std::endl << "Depth Threshold (Close/Far Points): " << m_thDepth << std::endl;
    }

    if(this->m_sensor==Selflocalization::RGBD)
    {
        m_depthMapFactor = std::stof(commandlineArgs["DepthMapFactor"]);
        if(std::fabs(m_depthMapFactor)<1e-5)
            m_depthMapFactor=1;
        else
            m_depthMapFactor = 1.0f/m_depthMapFactor;
    }
}

void Tracking::SetLocalMapper(std::shared_ptr<Mapping> pLocalMapper)
{
    m_localMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(std::shared_ptr<LoopClosing> pLoopClosing)
{
    m_loopClosing=pLoopClosing;
}

cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    m_imageGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(m_imageGray.channels()==3)
    {
        if(m_RGB)
        {
            cvtColor(m_imageGray,m_imageGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(m_imageGray,m_imageGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(m_imageGray.channels()==4)
    {
        if(m_RGB)
        {
            cvtColor(m_imageGray,m_imageGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(m_imageGray,m_imageGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }
   
    m_currentFrame = std::shared_ptr<OrbFrame>(new OrbFrame(m_imageGray,imGrayRight,timestamp,m_ORBextractorLeft,m_ORBextractorRight,m_ORBVocabulary,mK,m_distanceCoeffecient,m_bf,m_thDepth,m_boundingBox));

    Track();

    return m_currentFrame->mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    m_imageGray = imRGB;
    cv::Mat imDepth = imD;

    if(m_imageGray.channels()==3)
    {
        if(m_RGB)
            cvtColor(m_imageGray,m_imageGray,CV_RGB2GRAY);
        else
            cvtColor(m_imageGray,m_imageGray,CV_BGR2GRAY);
    }
    else if(m_imageGray.channels()==4)
    {
        if(m_RGB)
            cvtColor(m_imageGray,m_imageGray,CV_RGBA2GRAY);
        else
            cvtColor(m_imageGray,m_imageGray,CV_BGRA2GRAY);
    }

    if((fabs(m_depthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,m_depthMapFactor);

    m_currentFrame = std::shared_ptr<OrbFrame>(new OrbFrame(m_imageGray,imDepth,timestamp,m_ORBextractorLeft,m_ORBVocabulary,mK,m_distanceCoeffecient,m_bf,m_thDepth));

    Track();

    return m_currentFrame->mTcw.clone();
}


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    m_imageGray = im;

    if(m_imageGray.channels()==3)
    {
        if(m_RGB)
            cvtColor(m_imageGray,m_imageGray,CV_RGB2GRAY);
        else
            cvtColor(m_imageGray,m_imageGray,CV_BGR2GRAY);
    }
    else if(m_imageGray.channels()==4)
    {
        if(m_RGB)
            cvtColor(m_imageGray,m_imageGray,CV_RGBA2GRAY);
        else
            cvtColor(m_imageGray,m_imageGray,CV_BGRA2GRAY);
    }

    if(m_trackingState==NOT_INITIALIZED || m_trackingState==NO_IMAGES_YET)
        m_currentFrame = std::shared_ptr<OrbFrame>(new OrbFrame(m_imageGray,timestamp,m_initialORBextractor,m_ORBVocabulary,mK,m_distanceCoeffecient,m_bf,m_thDepth,m_boundingBox));
    else
        m_currentFrame = std::shared_ptr<OrbFrame>(new OrbFrame(m_imageGray,timestamp,m_ORBextractorLeft,m_ORBVocabulary,mK,m_distanceCoeffecient,m_bf,m_thDepth,m_boundingBox));

    Track();

    return m_currentFrame->mTcw.clone();
}

void Tracking::Track()
{
    if (!this->InitalizeTracking()) {
        std::cout << "Failed to initialize" << std::endl;
        return;
    }
    std::unique_lock<std::mutex> lock(m_orbMap->m_MapUpdateMutex);

    // System is initialized. Track Frame.
    bool bOK;

    // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
    if(!m_onlyTracking)
    {
        // Local Mapping is activated. This is the normal behaviour, unless
        // you explicitly activate the "only tracking" mode.
        if(m_trackingState==OK)
        {
            // Local Mapping might have changed some MapPoints tracked in last frame
            CheckReplacedInLastFrame();

            if(m_velocity.empty() || m_currentFrame->mnId<m_lastRelocationFrameId+2)
            {
                bOK = TrackReferenceKeyFrame();
                //std::cout << "Track reference keyframe is " << bOK << std::endl; 
            }
            else
            {
                bOK = TrackWithMotionModel();
                //std::cout << "Track with motion model is " << bOK << std::endl;
                if(!bOK){
                    bOK = TrackReferenceKeyFrame();
                    //std::cout << "Track reference keyframe is " << bOK << std::endl; 
                }

            }
        }
        else
        {
            bOK = Relocalization();
            std::cout << "Relocalizing is " << bOK << std::endl;
        }
    }
    else
    {
        // Localization Mode: Local Mapping is deactivated

        if(m_trackingState==LOST)
        {
            bOK = Relocalization();
        }
        else
        {
            if(!m_VO)
            {
                // In last frame we tracked enough MapPoints in the map

                if(!m_velocity.empty())
                {
                    bOK = TrackWithMotionModel();
                }
                else
                {
                    bOK = TrackReferenceKeyFrame();
                }
            }
            else
            {
                // In last frame we tracked mainly "visual odometry" points.

                // We compute two camera poses, one from motion model and one doing relocalization.
                // If relocalization is sucessfull we choose that solution, otherwise we retain
                // the "visual odometry" solution.

                bool bOKMM = false;
                bool bOKReloc = false;
                std::vector<std::shared_ptr<OrbMapPoint>> vpMPsMM;
                std::vector<bool> vbOutMM;
                cv::Mat TcwMM;
                if(!m_velocity.empty())
                {
                    bOKMM = TrackWithMotionModel();
                    vpMPsMM = m_currentFrame->m_mapPoints;
                    vbOutMM = m_currentFrame->m_outliers;
                    TcwMM = m_currentFrame->mTcw.clone();
                }
                bOKReloc = Relocalization();

                if(bOKMM && !bOKReloc)
                {
                    m_currentFrame->SetPose(TcwMM);
                    m_currentFrame->m_mapPoints = vpMPsMM;
                    m_currentFrame->m_outliers = vbOutMM;

                    if(m_VO)
                    {
                        for(int i =0; i<m_currentFrame->N; i++)
                        {
                            if(m_currentFrame->m_mapPoints[i] && !m_currentFrame->m_outliers[i])
                            {
                                m_currentFrame->m_mapPoints[i]->IncreaseFound();
                            }
                        }
                    }
                }
                else if(bOKReloc)
                {
                    m_VO = false;
                }

                bOK = bOKReloc || bOKMM;
            }
        }
    }

    m_currentFrame->m_referenceKeyFrame = m_referenceKeyFrame;

    // If we have an initial estimation of the camera pose and matching. Track the local map.
    if(!m_onlyTracking)
    {
        if(bOK){
            bOK = TrackLocalMap();
            std::cout << "Track local map is " << bOK << std::endl;
        }

    }
    else
    {
        // m_VO true means that there are few matches to MapPoints in the map. We cannot retrieve
        // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
        // the camera we will use the local map again.
        if(bOK && !m_VO){
            bOK = TrackLocalMap();
            //std::cout << "Track local map2 is " << bOK << std::endl;
        }
    }

    if(bOK)
        m_trackingState = OK;
    else
        m_trackingState=LOST;

    // Update drawer
    //mpFrameDrawer->Update(this);

    // If tracking were good, check if we insert a keyframe
    if(bOK)
    {
        // Update motion model
        if(!m_lastFrame->mTcw.empty())
        {
            cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
            m_lastFrame->GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
            m_lastFrame->GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
            m_velocity = m_currentFrame->mTcw*LastTwc;
        }
        else
            m_velocity = cv::Mat();

        //mpMapDrawer->SetCurrentCameraPose(m_currentFrame->mTcw);

        // Clean VO matches
        for(int i=0; i<m_currentFrame->N; i++)
        {
            std::shared_ptr<OrbMapPoint> pMP = m_currentFrame->m_mapPoints[i];
            if(pMP)
                if(pMP->GetObservingKeyFrameCount()<1)
                {
                    m_currentFrame->m_outliers[i] = false;
                    m_currentFrame->m_mapPoints[i]=static_cast<std::shared_ptr<OrbMapPoint>>(NULL);
                }
        }

        // Delete temporal MapPoints
        for(std::list<std::shared_ptr<OrbMapPoint>>::iterator lit = m_temporalPoints.begin(), lend =  m_temporalPoints.end(); lit!=lend; lit++)
        {
            std::shared_ptr<OrbMapPoint> pMP = *lit;
        }
        m_temporalPoints.clear();

        // Check if we need to insert a new keyframe
        if(NeedNewKeyFrame())
        {
            CreateNewKeyFrame();
        }

        // We allow points with high innovation (considererd outliers by the Huber Function)
        // pass to the new keyframe, so that bundle adjustment will finally decide
        // if they are outliers or not. We don't want next frame to estimate its position
        // with those points so we discard them in the frame.
        for(int i=0; i<m_currentFrame->N;i++)
        {
            if(m_currentFrame->m_mapPoints[i] && m_currentFrame->m_outliers[i])
                m_currentFrame->m_mapPoints[i]=static_cast<std::shared_ptr<OrbMapPoint>>(NULL);
        }
    }

    // Reset if the camera get lost soon after initialization
    if(m_trackingState==LOST)
    {
        if(m_orbMap->OrbKeyFramesCount()<=5)
        {
            std::cout << "Track lost soon after initialisation, reseting..." << std::endl;
            m_selflocalization->Reset();
            return;
        }
    }

    if(!m_currentFrame->m_referenceKeyFrame)
        m_currentFrame->m_referenceKeyFrame = m_referenceKeyFrame;

    m_lastFrame = std::shared_ptr<OrbFrame>(new OrbFrame(m_currentFrame));


    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!m_currentFrame->mTcw.empty())
    {
        cv::Mat Tcr = m_currentFrame->mTcw*m_currentFrame->m_referenceKeyFrame->GetPoseInverse();
        m_relativeFramePoses.push_back(Tcr);
        m_ReferenceKeyFrames.push_back(m_referenceKeyFrame);
        m_frameTimeStamps.push_back(m_currentFrame->mTimeStamp);
        m_Lost.push_back(m_trackingState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        m_relativeFramePoses.push_back(m_relativeFramePoses.back());
        m_ReferenceKeyFrames.push_back(m_ReferenceKeyFrames.back());
        m_frameTimeStamps.push_back(m_frameTimeStamps.back());
        m_Lost.push_back(m_trackingState==LOST);
    }

}


void Tracking::StereoInitialization()
{
    //std::cout << "Initializing Stereo" << std::endl;
    if(m_currentFrame->N>500)
    {
        // Set Frame pose to the origin
        m_currentFrame->SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        std::shared_ptr<OrbKeyFrame> pKFini = std::shared_ptr<OrbKeyFrame>(new OrbKeyFrame(m_currentFrame,m_orbMap,m_keyFrameDataBase));

        // Insert KeyFrame in the map
        m_orbMap->PushOrbKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
       for(int i=0; i<m_currentFrame->N;i++)
        {
            float z = m_currentFrame->m_depths[i];
            if(z>0)
            {
                cv::Mat x3D = m_currentFrame->UnprojectStereo(i);
                std::shared_ptr<OrbMapPoint> pNewMP = std::shared_ptr<OrbMapPoint>(new OrbMapPoint(x3D,pKFini,m_orbMap));
                pNewMP->AddObservingKeyframe(pKFini,i);
                pKFini->AddMapPoint(pNewMP,i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateMeanAndDepthValues();
                m_orbMap->PushOrbMapPoint(pNewMP);

                m_currentFrame->m_mapPoints[i]=pNewMP;
            }
        }
        
        std::cout << "New map created with " << m_orbMap->OrbMapPointsCount() << " points" << std::endl;

        m_localMapper->InsertKeyFrame(pKFini);

        m_lastFrame = std::shared_ptr<OrbFrame>(new OrbFrame(m_currentFrame));
        m_lastKeyFrameId= static_cast<unsigned int>(m_currentFrame->mnId);
        m_lastKeyFrame = pKFini;

        m_localKeyFrames.push_back(pKFini);
        m_localMapPoints=m_orbMap->GetAllMapPoints();
        m_referenceKeyFrame = pKFini;
        m_currentFrame->m_referenceKeyFrame = pKFini;

        m_orbMap->SetReferenceMapPoints(m_localMapPoints);

        m_orbMap->m_OrbKeyFrameOrigins.push_back(pKFini);

        //mpMapDrawer->SetCurrentCameraPose(m_currentFrame->mTcw);

        m_trackingState=OK;
    }
}

void Tracking::MonocularInitialization()
{
    //std::cout << "Initialize mono tracking" << std::endl;

    if(!m_initializer)
    {
        // Set Reference Frame
        if(m_currentFrame->m_keys.size()>100)
        {
            m_initialFrame = std::shared_ptr<OrbFrame>(new OrbFrame(m_currentFrame));
            m_lastFrame = std::shared_ptr<OrbFrame>(new OrbFrame(m_currentFrame));
            m_previouslyMatchedPoints.resize(m_currentFrame->m_undistortedKeys.size());
            for(size_t i=0; i<m_currentFrame->m_undistortedKeys.size(); i++)
                m_previouslyMatchedPoints[i]=m_currentFrame->m_undistortedKeys[i].pt;

            m_initializer = std::shared_ptr<OrbInitializer>(new OrbInitializer(m_currentFrame,1.0,200));

            fill(m_initialMatches.begin(),m_initialMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)m_currentFrame->m_keys.size()<=100)
        {
            m_initializer = static_cast<std::shared_ptr<OrbInitializer>>(NULL);
            fill(m_initialMatches.begin(),m_initialMatches.end(),-1);
            return;
        }
        //std::cout << "Initial frame keys: " << m_initialFrame->mvKeys.size() << "current frame keys: " << m_currentFrame->mvKeys.size() << std::endl;
        // Find correspondences
        ORBmatcher matcher(0.9f,true);
        int nmatches = matcher.SearchForInitialization(m_initialFrame,m_currentFrame,m_previouslyMatchedPoints,m_initialMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            m_initializer = static_cast<std::shared_ptr<OrbInitializer>>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        std::vector<bool> vbTriangulated; // Triangulated Correspondences (m_initialMatches)

        if(m_initializer->Initialize(m_currentFrame, m_initialMatches, Rcw, tcw, m_initialPoints3D, vbTriangulated))
        {
            for(size_t i=0, iend=m_initialMatches.size(); i<iend;i++)
            {
                if(m_initialMatches[i]>=0 && !vbTriangulated[i])
                {
                    m_initialMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            m_initialFrame->SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            m_currentFrame->SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    std::shared_ptr<OrbKeyFrame> pKFini = std::shared_ptr<OrbKeyFrame>(new OrbKeyFrame(m_initialFrame,m_orbMap,m_keyFrameDataBase));
    std::shared_ptr<OrbKeyFrame> pKFcur = std::shared_ptr<OrbKeyFrame>(new OrbKeyFrame(m_currentFrame,m_orbMap,m_keyFrameDataBase));


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    m_orbMap->PushOrbKeyFrame(pKFini);
    m_orbMap->PushOrbKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<m_initialMatches.size();i++)
    {
        if(m_initialMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(m_initialPoints3D[i]);
        std::shared_ptr<OrbMapPoint> pMP = std::shared_ptr<OrbMapPoint>(new OrbMapPoint(worldPos,pKFcur,m_orbMap));

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP, static_cast<const size_t &>(m_initialMatches[i]));

        pMP->AddObservingKeyframe(pKFini,i);
        pMP->AddObservingKeyframe(pKFcur, static_cast<size_t>(m_initialMatches[i]));

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateMeanAndDepthValues();

        //Fill Current Frame structure
        m_currentFrame->m_mapPoints[m_initialMatches[i]] = pMP;
        m_currentFrame->m_outliers[m_initialMatches[i]] = false;

        //Add to Map
        m_orbMap->PushOrbMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    std::cout << "New Map created with " << m_orbMap->OrbMapPointsCount() << " points" << std::endl;

    OrbOptimizer::GlobalBundleAdjustemnt(m_orbMap,20);
    //std::cout << "CurrPose :" << pKFcur->GetPose() << std::endl;

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        std::cout << "Wrong initialization, reseting..." << std::endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    std::vector<std::shared_ptr<OrbMapPoint>> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            std::shared_ptr<OrbMapPoint> pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPosition(pMP->GetWorldPosition()*invMedianDepth);
        }
    }

    m_localMapper->InsertKeyFrame(pKFini);
    m_localMapper->InsertKeyFrame(pKFcur);

    m_currentFrame->SetPose(pKFcur->GetPose());
    m_lastKeyFrameId= static_cast<unsigned int>(m_currentFrame->mnId);
    m_lastKeyFrame = pKFcur;

    m_localKeyFrames.push_back(pKFcur);
    m_localKeyFrames.push_back(pKFini);
    m_localMapPoints=m_orbMap->GetAllMapPoints();
    m_referenceKeyFrame = pKFcur;
    m_currentFrame->m_referenceKeyFrame = pKFcur;

    m_lastFrame = std::shared_ptr<OrbFrame>(new OrbFrame(m_currentFrame));

    m_orbMap->SetReferenceMapPoints(m_localMapPoints);

    //mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    m_orbMap->m_OrbKeyFrameOrigins.push_back(pKFini);

    m_trackingState=OK;
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<m_lastFrame->N; i++)
    {
        std::shared_ptr<OrbMapPoint> pMP = m_lastFrame->m_mapPoints[i];

        if(pMP)
        {
            std::shared_ptr<OrbMapPoint> pRep = pMP->GetReplaced();
            if(pRep)
            {
                m_lastFrame->m_mapPoints[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    m_currentFrame->ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7f,true);
    std::vector<std::shared_ptr<OrbMapPoint>> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(m_referenceKeyFrame,m_currentFrame,vpMapPointMatches);

    //std::cout << "nmatches " << nmatches << std::endl;
    if(nmatches<15)
        return false;

    m_currentFrame->m_mapPoints = vpMapPointMatches;
    m_currentFrame->SetPose(m_lastFrame->mTcw);
    //std::cout << "current pose " << m_currentFrame->mTcw << std::endl;
    OrbOptimizer::PoseOptimization(m_currentFrame);
    //std::cout << "return : " << iReturn << std::endl;

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<m_currentFrame->N; i++)
    {
        if(m_currentFrame->m_mapPoints[i])
        {
            if(m_currentFrame->m_outliers[i])
            {
                std::shared_ptr<OrbMapPoint> pMP = m_currentFrame->m_mapPoints[i];
                m_currentFrame->m_mapPoints[i]=static_cast<std::shared_ptr<OrbMapPoint>>(NULL);
                m_currentFrame->m_outliers[i]=false;
                pMP->SetTrackInView(false);
                pMP->SetLastFrameSeen(m_currentFrame->mnId);
                nmatches--;
            }
            else if(m_currentFrame->m_mapPoints[i]->GetObservingKeyFrameCount()>0)
                nmatchesMap++;
        }
    }
    return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    std::shared_ptr<OrbKeyFrame> pRef = m_lastFrame->m_referenceKeyFrame;
    cv::Mat Tlr = m_relativeFramePoses.back();

    m_lastFrame->SetPose(Tlr*pRef->GetPose());

    if(m_lastKeyFrameId==m_lastFrame->mnId || m_sensor==Selflocalization::MONOCULAR || !m_onlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    std::vector<std::pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(static_cast<unsigned long>(m_lastFrame->N));
    for(int i=0; i<m_lastFrame->N;i++)
    {
        float z = m_lastFrame->m_depths[i];
        if(z>0)
        {
            vDepthIdx.push_back(std::make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<m_thDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        std::shared_ptr<OrbMapPoint> pMP = m_lastFrame->m_mapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->GetObservingKeyFrameCount()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = m_lastFrame->UnprojectStereo(i);
            std::shared_ptr<OrbMapPoint> pNewMP = std::shared_ptr<OrbMapPoint>(new OrbMapPoint(x3D, m_lastFrame,m_orbMap,i));
            m_lastFrame->m_mapPoints[i]=pNewMP;

            m_temporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>m_thDepth && nPoints>100)
            break;
    }
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9f,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    m_currentFrame->SetPose(m_velocity*m_lastFrame->mTcw);

    fill(m_currentFrame->m_mapPoints.begin(),m_currentFrame->m_mapPoints.end(),static_cast<std::shared_ptr<OrbMapPoint>>(NULL));

    // Project points seen in previous frame
    int th;
    if(m_sensor!=Selflocalization::STEREO)
        th=15;
    else
        th=7;
    int nmatches = matcher.SearchByProjection(m_currentFrame,m_lastFrame,th,m_sensor==Selflocalization::MONOCULAR);
    //std::cout << "nmatches: " << nmatches << std::endl;
    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(m_currentFrame->m_mapPoints.begin(),m_currentFrame->m_mapPoints.end(),static_cast<std::shared_ptr<OrbMapPoint>>(NULL));
        nmatches = matcher.SearchByProjection(m_currentFrame,m_lastFrame,2*th,m_sensor==Selflocalization::MONOCULAR);
    }

    if(nmatches<20)
        return false;

    // Optimize frame pose with all matches
    OrbOptimizer::PoseOptimization(m_currentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<m_currentFrame->N; i++)
    {
        if(m_currentFrame->m_mapPoints[i])
        {
            if(m_currentFrame->m_outliers[i])
            {
                std::shared_ptr<OrbMapPoint> pMP = m_currentFrame->m_mapPoints[i];

                m_currentFrame->m_mapPoints[i]=static_cast<std::shared_ptr<OrbMapPoint>>(NULL);
                m_currentFrame->m_outliers[i]=false;
                pMP->SetTrackInView(static_cast<unsigned long>(false));
                pMP->SetLastFrameSeen(m_currentFrame->mnId);
                nmatches--;
            }
            else if(m_currentFrame->m_mapPoints[i]->GetObservingKeyFrameCount()>0)
                nmatchesMap++;
        }
    }

    if(m_onlyTracking)
    {
        m_VO = nmatchesMap<10;
        return nmatches>20;
    }

    return nmatchesMap>=10;
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    SearchLocalPoints();

    // Optimize Pose
    OrbOptimizer::PoseOptimization(m_currentFrame);
    m_matchesInliers = 0;
    int nRejectOutlier = 0;
    int nRejectUnseen = 0;
    // Update MapPoints Statistics
    for(int i=0; i<m_currentFrame->N; i++)
    {
        if(m_currentFrame->m_mapPoints[i])
        {
            if(!m_currentFrame->m_outliers[i])
            {
                m_currentFrame->m_mapPoints[i]->IncreaseFound();
                if(!m_onlyTracking)
                {
                    if(m_currentFrame->m_mapPoints[i]->GetObservingKeyFrameCount()>0)
                        m_matchesInliers++;
                }
                else
                    m_matchesInliers++;
            }
            else if(m_sensor==Selflocalization::STEREO)
                m_currentFrame->m_mapPoints[i] = static_cast<std::shared_ptr<OrbMapPoint>>(NULL);
            
            else
                nRejectOutlier++;
        }
        else
            nRejectUnseen++;
    }
    //std::cout << "Unseen: " << nRejectUnseen << "Outlier: " << nRejectOutlier << std::endl;
    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    //std::cout << "Inliers: " << m_matchesInliers << std::endl;
    if(m_currentFrame->mnId<m_lastRelocationFrameId+m_maxFrames && m_matchesInliers<50)
        return false;

    if(m_matchesInliers<30)
        return false;
    else
        return true;
}


bool Tracking::NeedNewKeyFrame()
{
    if(m_onlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(m_localMapper->isStopped() || m_localMapper->stopRequested())
        return false;

    const int nKFs = static_cast<const int>(m_orbMap->OrbKeyFramesCount());

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(m_currentFrame->mnId<m_lastRelocationFrameId+m_maxFrames && nKFs>m_maxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = m_referenceKeyFrame->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = m_localMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(m_sensor!=Selflocalization::MONOCULAR)
    {
        for(int i =0; i<m_currentFrame->N; i++)
        {
            if(m_currentFrame->m_depths[i]>0 && m_currentFrame->m_depths[i]<m_thDepth)
            {
                if(m_currentFrame->m_mapPoints[i] && !m_currentFrame->m_outliers[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(m_sensor==Selflocalization::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = m_currentFrame->mnId>=m_lastKeyFrameId+m_maxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (m_currentFrame->mnId>=m_lastKeyFrameId+m_minFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c =  m_sensor!=Selflocalization::MONOCULAR && (m_matchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((m_matchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && m_matchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            m_localMapper->InterruptBA();
            if(m_sensor!=Selflocalization::MONOCULAR)
            {
                if(m_localMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!m_localMapper->SetNotStop(true))
        return;

    std::shared_ptr<OrbKeyFrame> pKF = std::shared_ptr<OrbKeyFrame>(new OrbKeyFrame(m_currentFrame,m_orbMap,m_keyFrameDataBase));

    m_referenceKeyFrame = pKF;
    m_currentFrame->m_referenceKeyFrame = pKF;

    if(m_sensor!=Selflocalization::MONOCULAR)
    {
        m_currentFrame->UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < m_thDepth.
        // If there are less than 100 close points we create the 100 closest.
        std::vector<std::pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(static_cast<unsigned long>(m_currentFrame->N));
        for(int i=0; i<m_currentFrame->N; i++)
        {
            float z = m_currentFrame->m_depths[i];
            if(z>0)
            {
                vDepthIdx.push_back(std::make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                std::shared_ptr<OrbMapPoint> pMP = m_currentFrame->m_mapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->GetObservingKeyFrameCount()<1)
                {
                    bCreateNew = true;
                    m_currentFrame->m_mapPoints[i] = static_cast<std::shared_ptr<OrbMapPoint>>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = m_currentFrame->UnprojectStereo(i);
                    std::shared_ptr<OrbMapPoint> pNewMP = std::shared_ptr<OrbMapPoint>(new OrbMapPoint(x3D,pKF,m_orbMap));
                    pNewMP->AddObservingKeyframe(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateMeanAndDepthValues();
                    m_orbMap->PushOrbMapPoint(pNewMP);

                    m_currentFrame->m_mapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>m_thDepth && nPoints>100)
                    break;
            }
        }
    }

    m_localMapper->InsertKeyFrame(pKF);

    m_localMapper->SetNotStop(false);

    m_lastKeyFrameId = static_cast<unsigned int>(m_currentFrame->mnId);
    m_lastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(std::vector<std::shared_ptr<OrbMapPoint>>::iterator vit=m_currentFrame->m_mapPoints.begin(), vend=m_currentFrame->m_mapPoints.end(); vit!=vend; vit++)
    {
        std::shared_ptr<OrbMapPoint> pMP = *vit;
        if(pMP)
        {
            if(pMP->IsCorrupt())
            {
                *vit = static_cast<std::shared_ptr<OrbMapPoint>>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->SetLastFrameSeen(m_currentFrame->mnId);
                pMP->SetTrackInView(false);
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(std::vector<std::shared_ptr<OrbMapPoint>>::iterator vit=m_localMapPoints.begin(), vend=m_localMapPoints.end(); vit!=vend; vit++)
    {
        std::shared_ptr<OrbMapPoint> pMP = *vit;
        if(pMP->GetLastFrameSeen() == m_currentFrame->mnId)
            continue;
        if(pMP->IsCorrupt())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(m_currentFrame->IsInFrustum(pMP, 0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }
    //std::cout << "nToMatch: " << nToMatch << std::endl;
    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8f);
        int th = 1;
        if(m_sensor==Selflocalization::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(m_currentFrame->mnId<m_lastRelocationFrameId+2)
            th=5;
        matcher.SearchByProjection(m_currentFrame,m_localMapPoints,th);
        //std::cout << "local matches: " << matches << std::endl;
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    m_orbMap->SetReferenceMapPoints(m_localMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    m_localMapPoints.clear();

    for(std::vector<std::shared_ptr<OrbKeyFrame>>::const_iterator itKF=m_localKeyFrames.begin(), itEndKF=m_localKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        std::shared_ptr<OrbKeyFrame> pKF = *itKF;
        const std::vector<std::shared_ptr<OrbMapPoint>> vpMPs = pKF->GetMapPointMatches();

        for(std::vector<std::shared_ptr<OrbMapPoint>>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            std::shared_ptr<OrbMapPoint> pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->GetTrackReferenceForFrame()==m_currentFrame->mnId)
                continue;
            if(!pMP->IsCorrupt())
            {
                m_localMapPoints.push_back(pMP);
                pMP->SetTrackReferenceForFrame(m_currentFrame->mnId);
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    std::map<std::shared_ptr<OrbKeyFrame>,int> keyframeCounter;
    for(int i=0; i<m_currentFrame->N; i++)
    {
        if(m_currentFrame->m_mapPoints[i])
        {
            std::shared_ptr<OrbMapPoint> pMP = m_currentFrame->m_mapPoints[i];
            if(!pMP->IsCorrupt())
            {
                const std::map<std::shared_ptr<OrbKeyFrame>,size_t> observations = pMP->GetObservingKeyframes();
                for(std::map<std::shared_ptr<OrbKeyFrame>,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                m_currentFrame->m_mapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    std::shared_ptr<OrbKeyFrame> pKFmax= static_cast<std::shared_ptr<OrbKeyFrame>>(NULL);

    m_localKeyFrames.clear();
    m_localKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(std::map<std::shared_ptr<OrbKeyFrame>,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        std::shared_ptr<OrbKeyFrame> pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        m_localKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = m_currentFrame->mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(std::vector<std::shared_ptr<OrbKeyFrame>>::const_iterator itKF=m_localKeyFrames.begin(), itEndKF=m_localKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(m_localKeyFrames.size()>80)
            break;

        std::shared_ptr<OrbKeyFrame> pKF = *itKF;

        const std::vector<std::shared_ptr<OrbKeyFrame>> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(std::vector<std::shared_ptr<OrbKeyFrame>>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            std::shared_ptr<OrbKeyFrame> pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=m_currentFrame->mnId)
                {
                    m_localKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame = m_currentFrame->mnId;
                    break;
                }
            }
        }

        const std::set<std::shared_ptr<OrbKeyFrame>> spChilds = pKF->GetChilds();
        for(std::set<std::shared_ptr<OrbKeyFrame>>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            std::shared_ptr<OrbKeyFrame> pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=m_currentFrame->mnId)
                {
                    m_localKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame = m_currentFrame->mnId;
                    break;
                }
            }
        }

        std::shared_ptr<OrbKeyFrame> pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=m_currentFrame->mnId)
            {
                m_localKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame = m_currentFrame->mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        m_referenceKeyFrame = pKFmax;
        m_currentFrame->m_referenceKeyFrame = m_referenceKeyFrame;
    }
}

bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    m_currentFrame->ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    std::vector<std::shared_ptr<OrbKeyFrame>> vpCandidateKFs = m_keyFrameDataBase->DetectRelocalizationCandidates(m_currentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = static_cast<const int>(vpCandidateKFs.size());

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    std::vector<std::shared_ptr<PnPsolver>> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    std::vector<std::vector<std::shared_ptr<OrbMapPoint>> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    std::vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        std::shared_ptr<OrbKeyFrame> pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,m_currentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                std::shared_ptr<PnPsolver> pSolver = std::shared_ptr<PnPsolver>(new PnPsolver(m_currentFrame,vvpMapPointMatches[i]));
                pSolver->setRansacParameters(0.99,10,300,4,0.5f,5.991f);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9f,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            std::vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            std::shared_ptr<PnPsolver> pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(m_currentFrame->mTcw);

                std::set<std::shared_ptr<OrbMapPoint>> sFound;

                const int np = static_cast<const int>(vbInliers.size());

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        m_currentFrame->m_mapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        m_currentFrame->m_mapPoints[j]=NULL;
                }

                int nGood = OrbOptimizer::PoseOptimization(m_currentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<m_currentFrame->N; io++)
                    if(m_currentFrame->m_outliers[io])
                        m_currentFrame->m_mapPoints[io]=static_cast<std::shared_ptr<OrbMapPoint>>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(m_currentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = OrbOptimizer::PoseOptimization(m_currentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<m_currentFrame->N; ip++)
                                if(m_currentFrame->m_mapPoints[ip])
                                    sFound.insert(m_currentFrame->m_mapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(m_currentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = OrbOptimizer::PoseOptimization(m_currentFrame);

                                for(int io =0; io<m_currentFrame->N; io++)
                                    if(m_currentFrame->m_outliers[io])
                                        m_currentFrame->m_mapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        m_lastRelocationFrameId = static_cast<unsigned int>(m_currentFrame->mnId);
        return true;
    }

}

void Tracking::Reset()
{

    std::cout << "System Reseting" << std::endl;
//    if(mpViewer)
//    {
//        mpViewer->RequestStop();
//        while(!mpViewer->isStopped())
//            usleep(3000);
//    }

    // Reset Local Mapping
    std::cout << "Reseting Local Mapper...";
    m_localMapper->RequestReset();
    std::cout << " done" << std::endl;

    // Reset Loop Closing
    std::cout << "Reseting Loop Closing...";
    m_loopClosing->RequestReset();
    std::cout << " done" << std::endl;

    // Clear BoW Database
    std::cout << "Reseting Database...";
    m_keyFrameDataBase->Clear();
    std::cout << " done" << std::endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    m_orbMap->Reset();

    OrbKeyFrame::nNextId = 0;
    OrbFrame::m_nextId = 0;
    m_trackingState = NO_IMAGES_YET;

    if(m_initializer)
    {
        m_initializer = static_cast<std::shared_ptr<OrbInitializer>>(NULL);
    }

    m_relativeFramePoses.clear();
    m_ReferenceKeyFrames.clear();
    m_frameTimeStamps.clear();
    m_Lost.clear();

//   if(mpViewer)
//   mpViewer->Release();
}
void Tracking::ChangeCalibration(const std::string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(std::abs(k3)>0.0001f)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(m_distanceCoeffecient);

    m_bf = fSettings["Camera.bf"];

    OrbFrame::m_initialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    m_onlyTracking = flag;
}

bool Tracking::InitalizeTracking() {
    if(m_trackingState==NO_IMAGES_YET)
    {
        m_trackingState = NOT_INITIALIZED;
    }

    m_lastProcessedState=m_trackingState;

    // Get Map Mutex -> Map cannot be changed
    std::unique_lock<std::mutex> lock(m_orbMap->m_MapUpdateMutex);

    if(m_trackingState==NOT_INITIALIZED)
    {
        if(m_sensor==Selflocalization::STEREO || m_sensor==Selflocalization::RGBD)
            StereoInitialization();
        else
            MonocularInitialization();

        //mpFrameDrawer->Update(this);

        if(m_trackingState!=OK)
            return false;
    }
    return true;
}
void Tracking::WriteToPoseFile(const std::string &filename)
{
    std::cout << std::endl << "Saving camera trajectory to " << filename << " ..." << std::endl;
    std::ofstream f;
    f.open(filename.c_str());
    this->GetTrajectory(f);
    f.close();
    std::cout << std::endl << "trajectory saved!" << std::endl;
}

void Tracking::GetTrajectory(std::vector< std::pair<cv::Mat, cv::Mat>> & trajectory) {
    std::vector<std::shared_ptr<OrbKeyFrame>> keyFrames = m_orbMap->GetAllKeyFrames();
    std::sort(keyFrames.begin(),keyFrames.end(),OrbKeyFrame::FrameIDCompare);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = keyFrames[0]->GetPoseInverse();

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    std::list<std::shared_ptr<OrbKeyFrame>>::iterator lRit = this->m_ReferenceKeyFrames.begin();
    for(std::list<cv::Mat>::iterator lit = m_relativeFramePoses.begin(), lend = m_relativeFramePoses.end();lit!=lend;lit++, lRit++)
    {
        OrbKeyFrame * pKF = lRit->get();

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
            //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent().get();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat translationCameraWorld = (*lit)*Trw;
        cv::Mat rotationWorldCamera = translationCameraWorld.rowRange(0,3).colRange(0,3).t();
        cv::Mat translationWorldCamera = -rotationWorldCamera*translationCameraWorld.rowRange(0,3).col(3);
        trajectory.push_back(std::pair<cv::Mat, cv::Mat>(rotationWorldCamera, translationWorldCamera));
    }
}

void Tracking::GetTrajectory(std::ofstream &stream) {
    std::vector<std::shared_ptr<OrbKeyFrame>> keyFrames = m_orbMap->GetAllKeyFrames();
    std::sort(keyFrames.begin(),keyFrames.end(),OrbKeyFrame::FrameIDCompare);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = keyFrames[0]->GetPoseInverse();

    stream << std::fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    std::list<std::shared_ptr<OrbKeyFrame>>::iterator lRit = this->m_ReferenceKeyFrames.begin();
    for(std::list<cv::Mat>::iterator lit = m_relativeFramePoses.begin(), lend = m_relativeFramePoses.end();lit!=lend;lit++, lRit++)
    {
        OrbKeyFrame * pKF = lRit->get();

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
            //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent().get();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat translationCameraWorld = (*lit)*Trw;
        cv::Mat rotationWorldCamera = translationCameraWorld.rowRange(0,3).colRange(0,3).t();
        cv::Mat translationWorldCamera = -rotationWorldCamera*translationCameraWorld.rowRange(0,3).col(3);

        stream << std::setprecision(9) << rotationWorldCamera.at<float>(0,0) << " " << rotationWorldCamera.at<float>(0,1)  << " " << rotationWorldCamera.at<float>(0,2) << " "  << translationWorldCamera.at<float>(0) << " " <<
          rotationWorldCamera.at<float>(1,0) << " " << rotationWorldCamera.at<float>(1,1)  << " " << rotationWorldCamera.at<float>(1,2) << " "  << translationWorldCamera.at<float>(1) << " " <<
          rotationWorldCamera.at<float>(2,0) << " " << rotationWorldCamera.at<float>(2,1)  << " " << rotationWorldCamera.at<float>(2,2) << " "  << translationWorldCamera.at<float>(2) << std::endl;
    }
}
