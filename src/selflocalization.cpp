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

#include <ctype.h>
#include <cstring>
#include <cmath>
#include <iostream>
#include <iomanip>
#include "selflocalization.hpp"



/**
  * Constructor.
  *
  * @param a_argc Number of command line arguments.
  * @param a_argv Command line arguments.
  */
Selflocalization::Selflocalization(std::map<std::string, std::string> commandlineArgs) :
		  m_isMonocular()
		, m_pMapper()
		, m_pTracker()
		, m_pLoopCloser()
		, m_pMappingThread()
		, m_pLoopClosingThread()
		//, m_pImageGrab()
		, m_pExtractOrb()
		, m_pVocabulary()
		, m_pKeyFrameDatabase()
		, m_map()
		, m_last_envelope_ts()
		, m_resizeScale()

{
    if(commandlineArgs.count("kittiPath")>0)
    {
		setUp(commandlineArgs);
	}
	else
    {
		setUpRealtime(commandlineArgs);
	}
}

Selflocalization::~Selflocalization()
{
}

void Selflocalization::runKitti(std::string kittiPath)
{
	std::cout << "Starting Kittirunner" << std::endl;
	std::cout << kittiPath << std::endl;
	KittiRunner kittiRunner(kittiPath, !m_isMonocular, this, rmap);
   	cluon::OD4Session od4{static_cast<uint16_t>(m_cid), [](auto){}};
   	std::vector<long unsigned int> mapSizeVector;
   	std::vector<double> fpsVector;
	size_t lastMapPoint = 0;
	uint32_t lastSentIndex = 0;
    uint32_t lastSentCameraIndex = 0;
	for(size_t i = 0; i < kittiRunner.GetImagesCount(); i++ )
	{
        std::chrono::system_clock::time_point timePoint = std::chrono::system_clock::now();
		kittiRunner.ProcessImage(i,m_resizeScale);

        //std::cout << "Sending OD4" << std::endl;
		if(!m_isMonocular){
			std::pair<bool,opendlv::logic::sensation::Geolocation> posePacket = sendPose();
        	if(posePacket.first){
             	od4.send(posePacket.second,cluon::time::convert(timePoint),0);
			}
		}
		sendMap(lastMapPoint, lastSentCameraIndex, lastSentIndex, i, od4);
        // send results to conference.
        std::chrono::system_clock::time_point t2 = std::chrono::system_clock::now();
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - timePoint).count();
        fpsVector.push_back(ttrack);
        mapSizeVector.push_back(m_map->OrbMapPointsCount());
	}
	this->m_pTracker->WriteToPoseFile(kittiPath + "/poses.txt");
	writeToMapFile(kittiPath + "/map.txt");
	writeToFpsFile(kittiPath + "/fps.txt",mapSizeVector,fpsVector);
	kittiRunner.ShutDown();
}

void Selflocalization::writeToFpsFile(std::string filepath, std::vector<long unsigned int> mapSizeVector, std::vector<double> fpsVector)
{
	std::ofstream f;
    f.open(filepath.c_str());
    for(unsigned int i= 0; i<fpsVector.size(); i++){
    	f << fpsVector[i] << "\t" << mapSizeVector[i] << std::endl; 
    }
    f.close();
    std::cout << "Printed fps to file" << std::endl;
}

void Selflocalization::writeToMapFile(std::string filepath)
{
	if(m_map.get()){
		std::ofstream f;
    	f.open(filepath.c_str());
		auto mapPoints = m_map->GetAllMapPoints();
		for(unsigned int i = 0; i<mapPoints.size(); i++){
			OrbMapPoint* mp = mapPoints[i].get();
                if(mp->IsCorrupt())
                {
                    continue;
                }
                cv::Mat worldPosition = mp->GetWorldPosition();
                auto x = worldPosition.at<float>(0, 0);
                auto y = worldPosition.at<float>(1, 0);
                auto z = worldPosition.at<float>(2, 0);
				f << std::setprecision(9) << x << "\t" << y << "\t" << z << "\t" << std::endl;
		}
		f.close();
		std::cout << "map with " << m_map->OrbMapPointsCount() << " points saved" << std::endl;
	}

}

void Selflocalization::sendMap(size_t &lastMapPoint,uint32_t &lastSentCameraIndex, uint32_t &lastSentIndex, size_t i, cluon::OD4Session &od4)
{
	this->m_last_envelope_ts = std::chrono::steady_clock::now();
	OrbMap* map = m_map.get();

	std::stringstream cameraRotation;
	std::stringstream mappointCoordinates;
	std::stringstream cameraCoordinates;

	if(map && m_pTracker->GetTrackingState())
	{
        mappointCoordinates.str(std::string());
        cameraCoordinates.str(std::string());
        cameraRotation.str(std::string());

        cv::Mat R = m_pTracker->m_currentFrame->GetRotationInverse();
        cv::Mat Tcw = m_pTracker->m_currentFrame->mTcw;
        cv::Mat T = Tcw.rowRange(0, 3).colRange(3, 4);

        cv::Mat cameraPosition = -R * T;

        cameraCoordinates << std::fixed <<  std::setprecision(4) << cameraPosition.at<float>(0, 0) << ':';
        cameraCoordinates << std::fixed <<  std::setprecision(4) << cameraPosition.at<float>(1, 0) << ':';
        cameraCoordinates << std::fixed <<  std::setprecision(4) << cameraPosition.at<float>(2, 0) << ':';

        cameraRotation << std::fixed << std::setprecision(4) << R.at<float>(0, 2) << ':';
        cameraRotation << std::fixed << std::setprecision(4) << R.at<float>(1, 2) << ':';
        cameraRotation << std::fixed << std::setprecision(4) << R.at<float>(2, 2) << ':';

        int mapPointCount = 0;
        if(i%20 == 0)
        {
            lastMapPoint = 0;
            lastSentCameraIndex = 0;
            std::vector<std::pair<cv::Mat, cv::Mat>> trajectory;
            m_pTracker->GetTrajectory(trajectory);

            cameraCoordinates.str(std::string());
            int cameraPointCount = 0;
            for(auto cameraPoint: trajectory)
            {
                cameraCoordinates << std::fixed <<  std::setprecision(4) << cameraPoint.second.at<float>(0, 0) << ':';
                cameraCoordinates << std::fixed <<  std::setprecision(4) << cameraPoint.second.at<float>(1, 0) << ':';
                cameraCoordinates << std::fixed <<  std::setprecision(4) << cameraPoint.second.at<float>(2, 0) << ':';
                cameraPointCount++;
                if(cameraPointCount > 2500)
                {
                    opendlv::proxy::OrbslamMap orbSlamMap;
                    orbSlamMap.mapCoordinateIndex(lastSentIndex);
                    orbSlamMap.cameraCoordinateIndex(lastSentCameraIndex);
                    orbSlamMap.mapCoordinates(mappointCoordinates.str());
                    orbSlamMap.cameraCoordinates(cameraCoordinates.str());
                    orbSlamMap.cameraRotation(cameraRotation.str());
                    lastSentCameraIndex += cameraPointCount - 1;
                    std::chrono::system_clock::time_point timePoint = std::chrono::system_clock::now();
                    od4.send(orbSlamMap, cluon::time::convert(timePoint), i);
                    cameraCoordinates.str(std::string());
                    cameraPointCount = 0;
                }
            }
            opendlv::proxy::OrbslamMap orbSlamMap;
            orbSlamMap.mapCoordinateIndex(lastSentIndex);
            orbSlamMap.cameraCoordinateIndex(lastSentCameraIndex);
            orbSlamMap.mapCoordinates(mappointCoordinates.str());
            orbSlamMap.cameraCoordinates(cameraCoordinates.str());
            orbSlamMap.cameraRotation(cameraRotation.str());

            std::chrono::system_clock::time_point timePoint = std::chrono::system_clock::now();
            od4.send(orbSlamMap, cluon::time::convert(timePoint), i);
            cameraCoordinates.str(std::string());

            cameraCoordinates << std::fixed <<  std::setprecision(4) << cameraPosition.at<float>(0, 0) << ':';
            cameraCoordinates << std::fixed <<  std::setprecision(4) << cameraPosition.at<float>(1, 0) << ':';
            cameraCoordinates << std::fixed <<  std::setprecision(4) << cameraPosition.at<float>(2, 0) << ':';
        }
        lastSentCameraIndex = i;
        auto mapPoints = map->GetAllMapPoints();
        for(; lastMapPoint < mapPoints.size(); lastMapPoint++)
        {
            OrbMapPoint* mp = mapPoints[lastMapPoint].get();
            if(mp->IsCorrupt())
            {
                continue;
            }

            cv::Mat worldPosition = mp->GetWorldPosition();

            cv::Mat mapPointCameraPosition = Tcw.rowRange(0,3).colRange(0,3) * worldPosition + Tcw.rowRange(0,3).col(3);
            auto x = worldPosition.at<float>(0, 0);
            auto y = worldPosition.at<float>(1, 0);
            auto z = worldPosition.at<float>(2, 0);

            mappointCoordinates << std::fixed <<  std::setprecision(4) << x << ':';
            mappointCoordinates << std::fixed <<  std::setprecision(4) << y << ':';
            mappointCoordinates << std::fixed <<  std::setprecision(4) << z << ':';

            mapPointCount++;
            if(mapPointCount > 2500)
            {
                opendlv::proxy::OrbslamMap orbSlamMap;
                orbSlamMap.mapCoordinateIndex(lastSentIndex);
                orbSlamMap.cameraCoordinateIndex(lastSentCameraIndex);
                orbSlamMap.mapCoordinates(mappointCoordinates.str());
                orbSlamMap.cameraCoordinates(cameraCoordinates.str());
                orbSlamMap.cameraRotation(cameraRotation.str());

                std::chrono::system_clock::time_point timePoint = std::chrono::system_clock::now();
                od4.send(orbSlamMap, cluon::time::convert(timePoint), i);

                mappointCoordinates.str(std::string());
                lastSentIndex = (uint32_t)lastMapPoint - 1;
                mapPointCount = 0;
            }
        }

        opendlv::proxy::OrbslamMap orbSlamMap;
        orbSlamMap.mapCoordinateIndex(lastSentIndex);
        orbSlamMap.cameraCoordinateIndex(lastSentCameraIndex);
        orbSlamMap.mapCoordinates(mappointCoordinates.str());
        orbSlamMap.cameraCoordinates(cameraCoordinates.str());
        orbSlamMap.cameraRotation(cameraRotation.str());
        lastSentIndex = (uint32_t)lastMapPoint - 1;
        std::chrono::system_clock::time_point timePoint = std::chrono::system_clock::now();
        od4.send(orbSlamMap, cluon::time::convert(timePoint), i);
        mappointCoordinates.str(std::string());
    }
}

/*
*Takes data from conference, in our case image?
*/
void Selflocalization::nextContainer(cv::Mat &img)
{
	 cluon::data::TimeStamp currentTime = cluon::time::convert(std::chrono::system_clock::now());
	double currTime = currentTime.microseconds();

	if(!m_isMonocular){
		//std::cout << "Im in the stereo container" << std::endl;
		int width = img.cols;
		int height = img.rows;
		cv::Mat imgL(img, cv::Rect(0, 0, width/2, height));
		cv::Mat imgR(img, cv::Rect(width/2, 0, width/2, height));

		if(m_resizeScale < 1){

                cv::resize(imgL, imgL, cv::Size(static_cast<int>(imgL.cols*m_resizeScale),static_cast<int>(imgL.rows*m_resizeScale)));
                cv::resize(imgR, imgR, cv::Size(static_cast<int>(imgR.cols*m_resizeScale),static_cast<int>(imgR.rows*m_resizeScale)));
        }

		cv::remap(imgL,imgL, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
        cv::remap( imgR,imgR, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
		//GO TO TRACKING
		Track(imgL,imgR, currTime);
	}else{

		if(m_resizeScale < 1){

                cv::resize(img, img, cv::Size(static_cast<int>(img.cols*m_resizeScale),static_cast<int>(img.rows*m_resizeScale)));
        }

		Track(img,currTime);
	}
	sendPose();
}

std::pair<bool,opendlv::logic::sensation::Geolocation> Selflocalization::sendPose(){
	//Get the cameraPosition
	opendlv::logic::sensation::Geolocation poseMessage;
	if(!m_pTracker->GetTrackingState()){
		return std::pair<bool,opendlv::logic::sensation::Geolocation>(false,poseMessage);
	}
	cv::Mat R = m_pTracker->m_currentFrame->GetRotationInverse();
    cv::Mat T = m_pTracker->m_currentFrame->mTcw.rowRange(0, 3).col(3);
    cv::Mat cameraPosition = -R*T;
	//std::cout << cameraPosition << std::endl;
	double x = static_cast<double>(cameraPosition.at<float>(0,2));
	double y = -static_cast<double>(cameraPosition.at<float>(0,0));
	//double z = -static_cast<double>(cameraPosition.at<float>(0,1));
	//Convert to ENU frame
	//Rotate to ENU frame
	double newX=x*cos(m_referenceHeading)-y*sin(m_referenceHeading);
	double newY=y*cos(m_referenceHeading)+x*sin(m_referenceHeading);
	std::array<double,2> cartesianPos;
	//std::cout << "x: " << newX << "y: " << newY << std::endl;
  	cartesianPos[0] = newX;
  	cartesianPos[1] = newY;
  	std::array<double,2> sendGPS = wgs84::fromCartesian(m_gpsReference, cartesianPos);
  	poseMessage.longitude(static_cast<float>(sendGPS[1]));
  	poseMessage.latitude(static_cast<float>(sendGPS[0]));
	return std::pair<bool,opendlv::logic::sensation::Geolocation>(true,poseMessage);
	//Heading shift using a 2D rotation matrix
	//Then use the wgs84 reference to convert to geodetic coordinates and send
}

void Selflocalization::setUp(std::map<std::string, std::string> commandlineArgs)
{
	std::cout << "Setting up" << std::endl;
	m_isMonocular = std::stoi(commandlineArgs["cameraType"]) == 0;
	std::string vocFilePath = commandlineArgs["vocFilePath"]; //Create mount
	m_pVocabulary = std::shared_ptr<OrbVocabulary>(new OrbVocabulary(vocFilePath));
	//std::cout << "Size of Vocabulary: " << size << std::endl;
	//int colorChannel = 1;
	m_cid = std::stoi(commandlineArgs["cid"]);

	m_map = std::shared_ptr<OrbMap>(new OrbMap());
	//std::cout << "Created map" << std::endl;
	//m_pImageGrab = std::shared_ptr<ImageExtractor>(new ImageExtractor(colorChannel));
	/*int nFeatures = 1000;
    float scaleFactor = 1.2f;
    int nLevels = 8;
    int initialFastTh = 20;
    int minFastTh = 7;
    std::cout << "Hello" << std::endl;
    m_pExtractOrb = std::shared_ptr<OrbExtractor>(new OrbExtractor(nFeatures, scaleFactor, nLevels, initialFastTh, minFastTh));
    */
	const int sensor = std::stoi(commandlineArgs["cameraType"]);
    m_resizeScale = 1;


	m_pKeyFrameDatabase = std::shared_ptr<OrbKeyFrameDatabase>(new OrbKeyFrameDatabase(*m_pVocabulary.get()));
	std::cout << "Created keyframedatabase" << std::endl;

	m_pTracker = std::shared_ptr<Tracking>(new Tracking(std::shared_ptr<Selflocalization>(this),m_pVocabulary,m_map,m_pKeyFrameDatabase,commandlineArgs,sensor));
	std::cout << "Created Tracking" << std::endl;
	m_pMapper = std::shared_ptr<Mapping>(new Mapping(m_map,m_isMonocular));
	m_pMappingThread = std::shared_ptr<std::thread>(new std::thread(&Mapping::Run,m_pMapper));
	m_pMappingThread->detach();
	std::cout << "Created Mapping" << std::endl;

	m_pLoopCloser = std::shared_ptr<LoopClosing>(new LoopClosing(m_map,m_pKeyFrameDatabase,m_pVocabulary,!m_isMonocular));
	m_pLoopClosingThread = std::shared_ptr<std::thread>(new std::thread(&LoopClosing::Run,m_pLoopCloser));
	m_pLoopClosingThread->detach();
	std::cout << "Created Loop closing" << std::endl;
	m_pTracker->SetLocalMapper(m_pMapper);
	m_pTracker->SetLoopClosing(m_pLoopCloser);

	m_pMapper->SetTracker(m_pTracker);
	m_pMapper->SetLoopCloser(m_pLoopCloser);

	m_pLoopCloser->SetTracker(m_pTracker);
	m_pLoopCloser->SetLocalMapper(m_pMapper);
}


void Selflocalization::setUpRealtime(std::map<std::string, std::string> commandlineArgs)
{
	std::cout << "Setting up" << std::endl;
	m_gpsReference[0] = static_cast<double>(std::stod(commandlineArgs["refLatitude"]));
  	m_gpsReference[1] = static_cast<double>(std::stod(commandlineArgs["refLongitude"]));
	m_referenceHeading = static_cast<double>(std::stod(commandlineArgs["startHeading"]));
	m_referenceHeading = m_referenceHeading*DEG2RAD;
	m_referenceHeading = -m_referenceHeading+PI/2;

	m_referenceHeading = (m_referenceHeading > PI)?(m_referenceHeading-2*PI):(m_referenceHeading);

	m_referenceHeading = (m_referenceHeading < -PI)?(m_referenceHeading+2*PI):(m_referenceHeading);


	m_cid = std::stoi(commandlineArgs["cid"]);
	m_isMonocular = std::stoi(commandlineArgs["cameraType"]) == 0;
	std::string vocFilePath = commandlineArgs["vocFilePath"]; //Create mount
	m_pVocabulary = std::shared_ptr<OrbVocabulary>(new OrbVocabulary(vocFilePath));
	int size = m_pVocabulary->GetSize();
	std::cout << "Size of Vocabulary: " << size << std::endl;
	//int colorChannel = 1;

	m_map = std::shared_ptr<OrbMap>(new OrbMap());
	std::cout << "Created map" << std::endl;
	//m_pImageGrab = std::shared_ptr<ImageExtractor>(new ImageExtractor(colorChannel));
	/*int nFeatures = 1000;
    float scaleFactor = 1.2f;
    int nLevels = 8;
    int initialFastTh = 20;
    int minFastTh = 7;
    std::cout << "Hello" << std::endl;
    m_pExtractOrb = std::shared_ptr<OrbExtractor>(new OrbExtractor(nFeatures, scaleFactor, nLevels, initialFastTh, minFastTh));
    */
	const int sensor = std::stoi(commandlineArgs["cameraType"]);
	bool rectify = std::stoi(commandlineArgs["rectify"])==1;
	m_resizeScale= std::stof(commandlineArgs["resize"]);

	if(sensor && rectify){
		//Declare variables
		cv::Mat mtxLeft; 
        cv::Mat distLeft;
        cv::Mat mtxRight;
        cv::Mat distRight;
        cv::Mat R;
        cv::Mat rodrigues;
        cv::Mat Q;
        cv::Mat T;
        
        cv::Mat imgL;
        cv::Mat imgR;
        cv::Size stdSize; 

        cv::Mat R1;
        cv::Mat R2;
        cv::Mat P1;
        cv::Mat P2;
        cv::Rect validRoI[2];

		double resizeScale = static_cast<double>(m_resizeScale);
			//LEFT CAMERA PARAMETERS
		//m_resizeScale = static_cast<int>(resizeFloat);
		    float fx = std::stof(commandlineArgs["Camera.fx"]);
    		float fy = std::stof(commandlineArgs["Camera.fy"]);
    		float cx = std::stof(commandlineArgs["Camera.cx"]);
    		float cy = std::stof(commandlineArgs["Camera.cy"]);
			float k1 = std::stof(commandlineArgs["Camera.k1"]);
    		float k2 = std::stof(commandlineArgs["Camera.k2"]);
    		float k3 = std::stof(commandlineArgs["Camera.k3"]);
    		float p1 = std::stof(commandlineArgs["Camera.p1"]);
    		float p2 = std::stof(commandlineArgs["Camera.p2"]);

			//Left Camera
		    mtxLeft = (cv::Mat_<double>(3, 3) <<
        		fx*resizeScale, 0, cx*resizeScale,
        		0, fy*resizeScale, cy*resizeScale,
        		0, 0, 1);
    		distLeft = (cv::Mat_<double>(5, 1) << k1*resizeScale, k2*resizeScale, p1*resizeScale, p2*resizeScale, k3*resizeScale);

		//RIGHT CAMERA PARAMETERS
   			float fxr = std::stof(commandlineArgs["CameraR.fx"]);
    		float fyr = std::stof(commandlineArgs["CameraR.fy"]);
    		float cxr = std::stof(commandlineArgs["CameraR.cx"]);
    		float cyr = std::stof(commandlineArgs["CameraR.cy"]);
			float k1r = std::stof(commandlineArgs["CameraR.k1"]);
    		float k2r = std::stof(commandlineArgs["CameraR.k2"]);
    		float k3r = std::stof(commandlineArgs["CameraR.k3"]);
    		float p1r = std::stof(commandlineArgs["CameraR.p1"]);
    		float p2r = std::stof(commandlineArgs["CameraR.p2"]);
    		mtxRight = (cv::Mat_<double>(3, 3) <<
       			fxr*resizeScale, 0, cxr*resizeScale,
       			0, fyr*resizeScale, cyr*resizeScale,
       			0, 0, 1);
    		distRight = (cv::Mat_<double>(5, 1) << k1r*resizeScale, k2r*resizeScale, p1r*resizeScale, p2r*resizeScale, k3r*resizeScale);
		

		//COMMON PARAMETERS

		float rx = std::stof(commandlineArgs["Camera.rx"]);
    	float cv = std::stof(commandlineArgs["Camera.cv"]);
    	float rz = std::stof(commandlineArgs["Camera.rz"]); 
		float bs = std::stof(commandlineArgs["Camera.baseline"]);
		int width = std::stoi(commandlineArgs["width"]);
		int height = std::stoi(commandlineArgs["height"]);

		//RECTIFY IMAGES
    	T = (cv::Mat_<double>(3, 1) << -bs, 0, 0);
    	rodrigues = (cv::Mat_<double>(3, 1) << rx, cv, rz);
        cv::Rodrigues(rodrigues, R);
        stdSize = cv::Size(static_cast<int>(m_resizeScale*width/2), static_cast<int>(m_resizeScale*height));
        //stdSize = cv::Size((width/2), height);

        

        cv::stereoRectify(mtxLeft, distLeft, mtxRight, distRight, stdSize, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0.0, stdSize, &validRoI[0], &validRoI[1]);
        cv::initUndistortRectifyMap(mtxLeft, distLeft, R1, P1, stdSize, CV_16SC2, rmap[0][0], rmap[0][1]);
        cv::initUndistortRectifyMap(mtxRight, distRight, R2, P2, stdSize, CV_16SC2, rmap[1][0], rmap[1][1]);

		//New commandline arguments
		commandlineArgs["Camera.fx"] = std::to_string(P1.at<double>(0,0));
		commandlineArgs["Camera.fy"] = std::to_string(P1.at<double>(1,1));
		commandlineArgs["Camera.cx"] = std::to_string(P1.at<double>(0,2));
		commandlineArgs["Camera.cy"] = std::to_string(P1.at<double>(1,2));
		commandlineArgs["Camera.k1"] = std::to_string(0);
		commandlineArgs["Camera.k2"] = std::to_string(0);
		commandlineArgs["Camera.bf"] = std::to_string(-P2.at<double>(0,3));
		std::cout << P2.at<double>(0,3) << std::endl;
	}


	m_pKeyFrameDatabase = std::shared_ptr<OrbKeyFrameDatabase>(new OrbKeyFrameDatabase(*m_pVocabulary.get()));
	std::cout << "Created keyframedatabase" << std::endl;

	m_pTracker = std::shared_ptr<Tracking>(new Tracking(std::shared_ptr<Selflocalization>(this),m_pVocabulary,m_map,m_pKeyFrameDatabase,commandlineArgs,sensor));
	std::cout << "Created Tracking" << std::endl;
	m_pMapper = std::shared_ptr<Mapping>(new Mapping(m_map,m_isMonocular));
	m_pMappingThread = std::shared_ptr<std::thread>(new std::thread(&Mapping::Run,m_pMapper));
	m_pMappingThread->detach();
	std::cout << "Created Mapping" << std::endl;

	m_pLoopCloser = std::shared_ptr<LoopClosing>(new LoopClosing(m_map,m_pKeyFrameDatabase,m_pVocabulary,!m_isMonocular));
	m_pLoopClosingThread = std::shared_ptr<std::thread>(new std::thread(&LoopClosing::Run,m_pLoopCloser));
	m_pLoopClosingThread->detach();
	std::cout << "Created Loop closing" << std::endl;
	m_pTracker->SetLocalMapper(m_pMapper);
	m_pTracker->SetLoopClosing(m_pLoopCloser);

	m_pMapper->SetTracker(m_pTracker);
	m_pMapper->SetLoopCloser(m_pLoopCloser);

	m_pLoopCloser->SetTracker(m_pTracker);
	m_pLoopCloser->SetLocalMapper(m_pMapper);
}

void Selflocalization::Track(cv::Mat &imLeft, cv::Mat &imRight, double &timestamp){
	// Check reset
	{
		std::unique_lock<std::mutex> lock(mMutexReset);
		if(m_reset)
		{
			m_pTracker->Reset();
			m_reset = false;
		}
	}
	m_pTracker->GrabImageStereo(imLeft,imRight,timestamp);
}


void Selflocalization::Track(cv::Mat &imLeft, double &timestamp){
	// Check reset
	{
		std::unique_lock<std::mutex> lock(mMutexReset);
		if(m_reset)
		{
			m_pTracker->Reset();
			m_reset = false;
		}
	}
	m_pTracker->GrabImageMonocular(imLeft,timestamp);
}

void Selflocalization::Shutdown()
{
	m_pMapper->RequestFinish();
	m_pLoopCloser->RequestFinish();

	// Wait until all thread have effectively stopped
	while(!m_pMapper->isFinished() || !m_pLoopCloser->isFinished() || m_pLoopCloser->isRunningGBA())
	{
		usleep(5000);
	}
}


void Selflocalization::tearDown()
{
}

void Selflocalization::Reset(){
	std::unique_lock<std::mutex> lock(mMutexReset);
	m_reset = true;
}

opendlv::proxy::PointCloudReading Selflocalization::CreatePointCloudFromMap() {
	opendlv::proxy::PointCloudReading pointCloudPart1;
	pointCloudPart1.startAzimuth(0.0)
			.endAzimuth(0.0)
			.entriesPerAzimuth(12)
			.distances(std::string("hello"))
			.numberOfBitsForIntensity(0);
	return pointCloudPart1;
}

opendlv::proxy::OrbslamMap Selflocalization::sendToWebb(){
		std::stringstream mappointCoordinates;
		std::stringstream cameraCoordinates;
		size_t lastMapPoint = 0;

		OrbMap* map = m_map.get();
		if(map && m_pTracker->GetTrackingState())
		{
            mappointCoordinates.str(std::string());
            cameraCoordinates.str(std::string());

            cv::Mat R = m_pTracker->m_currentFrame->GetRotationInverse();
			
            cv::Mat T = m_pTracker->m_currentFrame->mTcw.rowRange(0, 3).col(3);

            cv::Mat cameraPosition = -R*T;

            cameraCoordinates << std::fixed <<  std::setprecision(4) << cameraPosition.at<float>(0, 0) << ':';
            cameraCoordinates << std::fixed <<  std::setprecision(4) << cameraPosition.at<float>(1, 0) << ':';
            cameraCoordinates << std::fixed <<  std::setprecision(4) << cameraPosition.at<float>(2, 0) << ':';

			auto mapPoints = map->GetAllMapPoints();
			for(; lastMapPoint < mapPoints.size(); lastMapPoint++)
			{
				OrbMapPoint* mp = mapPoints[lastMapPoint].get();
				cv::Mat worldPosition = mp->GetWorldPosition();
				auto x = worldPosition.at<float>(0, 0);
				auto y = worldPosition.at<float>(1, 0);
				auto z = worldPosition.at<float>(2, 0);

				mappointCoordinates << std::fixed <<  std::setprecision(4) << x << ':';
				mappointCoordinates << std::fixed <<  std::setprecision(4) << y << ':';
				mappointCoordinates << std::fixed <<  std::setprecision(4) << z << ':';
			}
		}


        std::cout << "Length of mapPoints is: " << mappointCoordinates.str().length() << "." << std::endl;
		opendlv::proxy::OrbslamMap orbSlamMap;
		orbSlamMap.mapCoordinates(mappointCoordinates.str());
		orbSlamMap.cameraCoordinates(cameraCoordinates.str());

		return orbSlamMap;
        // send results to conference.
}