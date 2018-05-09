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

//#include "opendavinci/odcore/data/Container.h"
//#include "opendavinci/odcore/data/TimeStamp.h"

//#include "odvdopendlvdata/GeneratedHeaders_ODVDOpenDLVData.h"

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

{
    this->m_last_envelope_ts = std::chrono::steady_clock::now();
	setUp(commandlineArgs);


}

Selflocalization::~Selflocalization()
{
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
		cv::remap(imgL,imgL, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
        cv::remap( imgR,imgR, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
		//GO TO TRACKING
		Track(imgL,imgR, currTime);

	
	}else{
		Track(img,currTime);

	}

}

void Selflocalization::setUp(std::map<std::string, std::string> commandlineArgs)
{
	std::cout << "Setting up" << std::endl;
	m_isMonocular = std::stoi(commandlineArgs["cameraType"]) == 0;
	std::string vocFilePath = commandlineArgs["vocFilePath"]; //Create mount
	m_pVocabulary = std::shared_ptr<OrbVocabulary>(new OrbVocabulary(vocFilePath));
	int size = m_pVocabulary->getSize();
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

	if(sensor){
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
		//Grab parameters
		    float fx = std::stof(commandlineArgs["Camera.fx"]);
    		float fy = std::stof(commandlineArgs["Camera.fy"]);
    		float cx = std::stof(commandlineArgs["Camera.cx"]);
    		float cy = std::stof(commandlineArgs["Camera.cy"]);
			float k1 = std::stof(commandlineArgs["Camera.k1"]);
    		float k2 = std::stof(commandlineArgs["Camera.k2"]);
    		float k3 = std::stof(commandlineArgs["Camera.k3"]);
    		float p1 = std::stof(commandlineArgs["Camera.p1"]);
    		float p2 = std::stof(commandlineArgs["Camera.p2"]);
    		float rx = std::stof(commandlineArgs["Camera.rx"]);
    		float cv = std::stof(commandlineArgs["Camera.cv"]);
    		float rz = std::stof(commandlineArgs["Camera.rz"]); 
			float bs = std::stof(commandlineArgs["Camera.baseline"]);
			int width = std::stoi(commandlineArgs["width"]);
			int height = std::stoi(commandlineArgs["height"]);
			//Left Camera
		    mtxLeft = (cv::Mat_<double>(3, 3) <<
        		fx, 0, cx,
        		0, fy, cy,
        		0, 0, 1);
    		distLeft = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);
			//Right Camera
    		mtxRight = (cv::Mat_<double>(3, 3) <<
       			700.225, 0, 660.759,
       			0, 700.225, 364.782,
       			0, 0, 1);
    		distRight = (cv::Mat_<double>(5, 1) << -0.174209, 0.026726, 0, 0, 0);

    	T = (cv::Mat_<double>(3, 1) << -bs, 0, 0);
    	rodrigues = (cv::Mat_<double>(3, 1) << -rx, cv, rz);
        cv::Rodrigues(rodrigues, R);
        stdSize = cv::Size(width/2, height);

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
