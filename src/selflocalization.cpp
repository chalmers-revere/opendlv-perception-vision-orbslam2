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
Selflocalization::Selflocalization() :
      m_cameraType()
    , m_pMapper()
    , m_pTracker()
    //, m_pImageGrab()
    , m_pExtractOrb()
    , m_pVocabulary()
    , m_map()

{	
	
  setUp();
	/*
	m_pVocabulary = new OrbVocabulary();
	m_pVocabulary->loadFromTextFile(vocFilePath);

	m_pKeyFrameDatabase = new KeyFrameDatabase(m_pVocabulary);

	m_pMap = new Map();

	
	
	m_pMapper = new Mapping(m_pMap);
	m_pMappingThread = new Thread(Mapping::Run(),m_pMapper);	

	
	m_pLoopCloser = new LoopClosing(m_pVocabulary,m_pMap,m_pKeyFrameDatabase);
	m_pLoopClosingThread = new Thread(LoopClosing::Run(),m_pLoopCloser);

	mpTracker->SetLocalMapper(m_pMapper);
	mpTracker->SetLoopClosing(m_pLoopCloser);

	mpLocalMapper->SetTracker(m_pTracker);
	mpLocalMapper->SetLoopCloser(m_pLoopCloser);

	mpLoopCloser->SetTracker(m_pTracker);
	mpLoopCloser->SetLocalMapper(m_pMapper);
	*/
	
	//Initialization
	
	//Orb vocabulary - global pointer
	//KeyframDatabase - global pointer
	//Map - global pointer
	//Tracking - global pointer
	//Local mapping - global pointer
	//Loop closing - global pointer

	//System threads
	//Tracking lives in system thread
	//Local mapping - global thread pointer
	//Loop closing - global thread pointer




	/*System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               		const bool bUseViewer):mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),mbActivateLocalizationMode(false),
					mbDeactivateLocalizationMode(false)*/
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


	/*  	opendlv::proxy::ImageReadingShared sharedImg = cluon::extractMessage<opendlv::proxy::ImageReadingShared>(std::move(a_container));


		img = m_pImageGrab->ExtractSharedImage(&sharedImg);

	if(m_cameraType){
		int width = img.cols;
		int height = img.rows;
		cv::Mat imgL(img, cv::Rect(0, 0, width/2, height));
		cv::Mat imgR(img, cv::Rect(width/2, 0, width/2, height));
		//GO TO TRACKING

	cv::Mat m_cameraPose = m_pImageGrab->ImageToGreyscaleStereo(imgL,imgR,currentTime);

	*/
	//}else{
	  //GO TO TRACKING
	  //cv::Mat m_cameraPose = m_pImageGrab->ImageToGreyscaleMono(img,currentTime);
	/*ORB testcode*/
	std::vector<cv::KeyPoint> TestMat;
	cv::Mat testArr;
	cv::Mat Tcw = cv::imread("/media/test2.jpg",CV_LOAD_IMAGE_COLOR);
	cv::cvtColor(Tcw,Tcw,cv::COLOR_RGB2GRAY);
	cv::Mat Tcw_keypoints;
	m_pExtractOrb->ExtractFeatures(Tcw, TestMat, testArr);
	cv::drawKeypoints( Tcw, TestMat, Tcw_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
	cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
	cv::imshow( "Display window", Tcw_keypoints );                   // Show our image inside it.
	cv::waitKey(0);

	//}

			//std::cout << "[" << getName() << "] " << "[Unable to extract shared image." << std::endl;
	//}

	//if stereo
}

void Selflocalization::setUp()
{
	//odcore::base::KeyValueConfiguration kv = getKeyValueConfiguration();
    	//m_cameraType = (kv.getValue<int32_t>("logic-sensation-selflocalization.cameratype") == 1);
	m_cameraType = true;

	//string vocFilePath = kv.getValue<string>("logic-sensation-selflocalization.vocabularyfilepath");

	std::string vocFilePath = "/media/ORBvoc.txt"; //Create mount
	m_pVocabulary = std::shared_ptr<OrbVocabulary>(new OrbVocabulary(vocFilePath));
	int size = m_pVocabulary->getSize();
	std::cout << "Size of Vocabulary: " << size << std::endl;
	//int colorChannel = 1;

	//m_pTracker = std::shared_ptr<Tracking>(new Tracking(std::shared_ptr<Selflocalization>(this), colorChannel /*,m_pVocavulary,m_pKeyFrameDatabase,m_pMap*/));

	m_map = std::shared_ptr<OrbMap>(new OrbMap());
	m_pMapper = std::shared_ptr<Mapping>(new Mapping(m_map,m_cameraType));
    //m_pImageGrab = std::shared_ptr<ImageExtractor>(new ImageExtractor(colorChannel));
    int nFeatures = 1000;
    float scaleFactor = 1.2f;
    int nLevels = 8;
    int initialFastTh = 20;
    int minFastTh = 7;
    std::cout << "Hello" << std::endl;
    m_pExtractOrb = std::shared_ptr<OrbExtractor>(new OrbExtractor(nFeatures, scaleFactor, nLevels, initialFastTh, minFastTh));
	
	/*
	m_pKeyFrameDatabase = new KeyFrameDatabase(m_pVocabulary);
	m_pMap = new Map();
	m_pMappingThread = new Thread(Mapping::Run(),m_pMapper);
	m_pLoopCloser = new LoopClosing(m_pVocabulary,m_pMap,m_pKeyFrameDatabase);
	m_pLoopClosingThread = new Thread(LoopClosing::Run(),m_pLoopCloser);

	mpTracker->SetLocalMapper(m_pMapper);
	mpTracker->SetLoopClosing(m_pLoopCloser);

	mpLocalMapper->SetTracker(m_pTracker);
	mpLocalMapper->SetLoopCloser(m_pLoopCloser);

	mpLoopCloser->SetTracker(m_pTracker);
	mpLoopCloser->SetLocalMapper(m_pMapper);
	*/
	

}

void Selflocalization::tearDown()
{
}

