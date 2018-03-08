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

#ifndef IMAGEEXTRACTOR_HPP
#define IMAGEEXTRACTOR_HPP

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
//#include <opendavinci/odcore/data/TimeStamp.h>
//#include <opendavinci/odcore/strings/StringToolbox.h>
//#include <opendavinci/odcore/wrapper/Eigen.h>
//#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
//#include <opendavinci/odcore/data/Container.h>
//#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
//#include "opendavinci/odcore/wrapper/SharedMemoryFactory.h"
//#include "opendavinci/odcore/wrapper/SharedMemory.h"
//#include "opendavinci/generated/odcore/data/CompactPointCloud.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include "opendavinci/odcore/base/KeyValueConfiguration.h"
//#include "selflocalization.hpp"

class ImageExtractor
 {
 public:
  ImageExtractor(int colourChannel);
  ImageExtractor(ImageExtractor const &) = delete;
  ImageExtractor &operator=(ImageExtractor const &) = delete;
  virtual ~ImageExtractor();
  cv::Mat ImageToGreyscaleStereo(cv::Mat &imgL, cv::Mat &imgR, double &timeStamp);
  cv::Mat ImageToGreyscaleMono(cv::Mat &img, double &timeStamp);
  cv::Mat ExtractSharedImage(opendlv::proxy::ImageReadingShared *a_sharedImage);
  void saveImg(cv::Mat &img);

 private:
 cv::Mat m_imGrey;
 bool m_RGB; //Order of colour channels
 int m_saveCounter = 0;
	
};


#endif
