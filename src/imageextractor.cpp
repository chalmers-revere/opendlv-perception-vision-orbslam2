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

//#include "opendavinci/odcore/data/TimeStamp.h"

#include "imageextractor.hpp"


/**
  * Constructor.
  *
  * @param a_argc Number of command line arguments.
  * @param a_argv Command line arguments.
  */
ImageExtractor::ImageExtractor(int colorChannel/*, std::shared_ptr<OrbVocabulary> m_pVocabulary, std::shared_ptr<KeyFrameDatabase> m_pKeyFrameDatabase, std::shared_ptr<Map> m_pMap*/)
: m_imGrey()
, m_RGB()
, m_saveCounter()
{
  
	if(colorChannel == 1){
		m_RGB = true;
	}else{
		m_RGB = false;
	}
}

cv::Mat ImageExtractor::ImageToGreyscaleStereo(cv::Mat &imgL, cv::Mat &imgR, double &timeStamp)
{
	m_imGrey = imgL;
	cv::Mat imGrayRight = imgR;
	timeStamp = timeStamp;

	if(m_imGrey.channels() == 3){

		if(m_RGB){

			cv::cvtColor(m_imGrey,m_imGrey,cv::COLOR_RGB2GRAY);
			cv::cvtColor(imGrayRight,imGrayRight,cv::COLOR_RGB2GRAY);
			
		}else{

			cv::cvtColor(m_imGrey,m_imGrey,cv::COLOR_BGR2GRAY);
			cv::cvtColor(imGrayRight,imGrayRight,cv::COLOR_BGR2GRAY);

		}

	}else if(m_imGrey.channels() == 4){ //In case of Alpha channel


		if(m_RGB){

			cv::cvtColor(m_imGrey,m_imGrey,cv::COLOR_RGBA2GRAY);
			cv::cvtColor(imGrayRight,imGrayRight,cv::COLOR_RGBA2GRAY);
			
		}else{

			cv::cvtColor(m_imGrey,m_imGrey,cv::COLOR_BGRA2GRAY);
			cv::cvtColor(imGrayRight,imGrayRight,cv::COLOR_BGRA2GRAY);

		}

	}

	saveImg(m_imGrey);
	saveImg(imGrayRight);

	/*mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

	return mCurrentFrame.mTcw.clone();*/



	return m_imGrey; //CHANGE RETURN TO FRAME

}

cv::Mat ImageExtractor::ImageToGreyscaleMono(cv::Mat &img, double &timeStamp)
{
	m_imGrey = img;
	timeStamp = timeStamp;

	if(m_imGrey.channels() == 3){

		if(m_RGB){

			cv::cvtColor(m_imGrey,m_imGrey,cv::COLOR_RGB2GRAY);
			
		}else{

			cv::cvtColor(m_imGrey,m_imGrey,cv::COLOR_BGR2GRAY);

		}

	}else if(m_imGrey.channels() == 4){ //In case of Alpha channel


		if(m_RGB){

			cv::cvtColor(m_imGrey,m_imGrey,cv::COLOR_RGBA2GRAY);
			
		}else{

			cv::cvtColor(m_imGrey,m_imGrey,cv::COLOR_BGRA2GRAY);

		}

	}

	saveImg(m_imGrey);
	/*mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

	return mCurrentFrame.mTcw.clone();*/



	return m_imGrey; //CHANGE RETURN TO FRAME

}

cv::Mat ImageExtractor::ExtractSharedImage(opendlv::proxy::ImageReadingShared *a_sharedImage)
{
  
  cv::Mat img;
  std::shared_ptr<cluon::wrapper::SharedMemory> sharedMem(cluon::wrapper::SharedMemoryFactory::attachToSharedMemory(a_sharedImage->name()));
  if (sharedMem->isValid()) {
    const uint32_t nrChannels = a_sharedImage->bytesPerPixel();
    const uint32_t imgWidth = a_sharedImage->width();
    const uint32_t imgHeight = a_sharedImage->height();
    IplImage* myIplImage = cvCreateImage(cvSize(imgWidth,imgHeight), IPL_DEPTH_8U, nrChannels);
    cv::Mat bufferImage = cv::Mat(myIplImage);
    	{
      		sharedMem->lock();
      		memcpy(bufferImage.data, sharedMem->sharedMemory()
        	, imgWidth*imgHeight*nrChannels);
      		sharedMem->unlock();
    	}
    	img.release();
    	img = bufferImage.clone();
    	bufferImage.release();
    	cvReleaseImage(&myIplImage);
   
  }else{

    //std::cout << "[" << getName() << "] " << "Sharedmem is not valid." << std::endl;
    std::cout << "[" << "getName()" << "] " << "Sharedmem is not valid." << std::endl;
  }

  return img;
}

void ImageExtractor::saveImg(cv::Mat &img) {
	if(m_saveCounter < 50){
  		std::string img_name = std::to_string(m_saveCounter);
  		cv::imwrite("Images/"+img_name+".png", img);
		m_saveCounter++;
	}
}

ImageExtractor::~ImageExtractor()
{
}

