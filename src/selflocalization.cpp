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
		, m_map(),
        m_last_envelope_ts()

{
    this->m_last_envelope_ts = std::chrono::steady_clock::now();
	setUp(commandlineArgs);
	std::cout << "Starting Kittirunner" << std::endl;
	std::cout << commandlineArgs["kittiPath"] << std::endl;
	KittiRunner kittiRunner(commandlineArgs["kittiPath"],!m_isMonocular,std::shared_ptr<Selflocalization>(this));

    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArgs["cid"])), [](auto){}};

	std::stringstream mappointCoordinates;
	std::stringstream cameraCoordinates;
    std::stringstream cameraRotation;

//	size_t lastMapPoint = 0;
//    size_t lastLoopClosingId = 0;

	for(size_t i = 0; i < kittiRunner.GetImagesCount(); i++ )
	{
		kittiRunner.ProcessImage(i);
		OrbMap* map = m_map.get();
		if(map)
		{
            mappointCoordinates.str(std::string());
            cameraCoordinates.str(std::string());
            cameraRotation.str(std::string());



            cv::Mat R = m_pTracker->mCurrentFrame->GetRotationInverse();
            cv::Mat Tcw = m_pTracker->mCurrentFrame->mTcw;
            cv::Mat T = Tcw.rowRange(0, 3).colRange(3, 4);

            cv::Mat cameraPosition = -R * T;

            cameraCoordinates << std::fixed <<  std::setprecision(4) << cameraPosition.at<float>(0, 0) << ':';
            cameraCoordinates << std::fixed <<  std::setprecision(4) << cameraPosition.at<float>(1, 0) << ':';
            cameraCoordinates << std::fixed <<  std::setprecision(4) << cameraPosition.at<float>(2, 0) << ':';

            cameraRotation << std::fixed << std::setprecision(4) << R.at<float>(0, 2) << ':';
            cameraRotation << std::fixed << std::setprecision(4) << R.at<float>(1, 2) << ':';
            cameraRotation << std::fixed << std::setprecision(4) << R.at<float>(2, 2) << ':';


            uint32_t lastSentIndex = 0;
            int mapPointCount = 0;

			auto mapPoints = map->GetAllMapPoints();
			for(size_t j = 0; j < mapPoints.size(); j++)
			{
				OrbMapPoint* mp = mapPoints[j].get();
				cv::Mat worldPosition = mp->GetWorldPosition();

                cv::Mat mapPointCameraPosition = Tcw.rowRange(0,3).colRange(0,3) * worldPosition + Tcw.rowRange(0,3).col(3);
				auto x = worldPosition.at<float>(0, 0);
				auto y = worldPosition.at<float>(1, 0);
				auto z = worldPosition.at<float>(2, 0);

				mappointCoordinates << std::fixed <<  std::setprecision(4) << x << ':';
				mappointCoordinates << std::fixed <<  std::setprecision(4) << y << ':';
				mappointCoordinates << std::fixed <<  std::setprecision(4) << z << ':';

                mapPointCount++;
                if(mapPointCount > 1000)
                {
                    opendlv::proxy::OrbslamMap orbSlamMap;
                    orbSlamMap.mapCoordinateIndex(lastSentIndex);
                    orbSlamMap.mapCoordinates(mappointCoordinates.str());
                    orbSlamMap.cameraCoordinates(cameraCoordinates.str());
                    orbSlamMap.cameraRotation(cameraRotation.str());

                    std::chrono::system_clock::time_point timePoint = std::chrono::system_clock::now();
                    od4.send(orbSlamMap, cluon::time::convert(timePoint), i);

                    mappointCoordinates.str(std::string());
                    lastSentIndex = static_cast<uint32_t>(mapPointCount) - 1;
                    mapPointCount = 0;
                }
			}

            opendlv::proxy::OrbslamMap orbSlamMap;
            orbSlamMap.mapCoordinateIndex(lastSentIndex);
            orbSlamMap.mapCoordinates(mappointCoordinates.str());
            orbSlamMap.cameraCoordinates(cameraCoordinates.str());
            orbSlamMap.cameraRotation(cameraRotation.str());

            std::chrono::system_clock::time_point timePoint = std::chrono::system_clock::now();
            od4.send(orbSlamMap, cluon::time::convert(timePoint), i);

            if(i == 3)
            {
                this->m_pTracker->WriteToPoseFile("poses.txt");
            }
            mappointCoordinates.str(std::string());
		}

        // send results to conference.
	}
	this->m_pTracker->WriteToPoseFile("poses.txt");
	kittiRunner.ShutDown();
}

Selflocalization::~Selflocalization()
{
}
/*
*Takes data from conference, in our case image?
*/
void Selflocalization::nextContainer(cluon::data::Envelope &a_container)
{
	//cv::Mat img;

	//if (a_container.dataType() == opendlv::proxy::ImageReadingShared::ID()){

	cluon::data::TimeStamp currTime = a_container.sampleTimeStamp();
	double currentTime = currTime.microseconds();
	std::cout << "CurrentTime: " << currentTime << std::endl;


	/*opendlv::proxy::ImageReadingShared sharedImg = cluon::extractMessage<opendlv::proxy::ImageReadingShared>(std::move(a_container));


		img = m_pImageGrab->ExtractSharedImage(&sharedImg);

	if(m_cameraType){
		int width = img.cols;
		int height = img.rows;
		cv::Mat imgL(img, cv::Rect(0, 0, width/2, height));
		cv::Mat imgR(img, cv::Rect(width/2, 0, width/2, height));
		//GO TO TRACKING

	    //cv::Mat m_cameraPose = m_pImageGrab->ImageToGreyscaleStereo(imgL,imgR,currentTime);
		//cv::Mat m_cameraPose = m_pTracker->GrabImageStereo(imgL,imgR,timestamp);

	*/
	//}else{
	//GO TO TRACKING
	//cv::Mat m_cameraPose = m_pImageGrab->ImageToGreyscaleMono(img,currentTime);
	//cv::Mat m_cameraPose = m_pTracker->GrabImageMonocular(img,currentTime);
	/*ORB testcode
	std::vector<cv::KeyPoint> TestMat;
	cv::Mat testArr;
	cv::Mat Tcw = cv::imread("/media/test2.jpg",CV_LOAD_IMAGE_COLOR);
	cv::cvtColor(Tcw,Tcw,cv::COLOR_RGB2GRAY);
	cv::Mat Tcw_keypoints;
	m_pExtractOrb->ExtractFeatures(Tcw, TestMat, testArr);
	cv::drawKeypoints( Tcw, TestMat, Tcw_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
	cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
	cv::imshow( "Display window", Tcw_keypoints );                   // Show our image inside it.
	cv::waitKey(0);*/

	//}

	//std::cout << "[" << getName() << "] " << "[Unable to extract shared image." << std::endl;
	//}

	//if stereo
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