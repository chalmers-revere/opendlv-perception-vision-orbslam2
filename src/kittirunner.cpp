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

#include "kittirunner.hpp"

KittiRunner::KittiRunner(const std::string &kittiPath,bool isStereo,std::shared_ptr<Selflocalization> slammer):
        m_imagesCount(0), m_isStereo(isStereo), m_slammer(slammer), m_leftImages(), m_rightImages(), m_timeStamps(), m_timeTrackingStatistics(){
//    std::vector<std::string> vstrImageLeft;
//    std::vector<std::string> vstrImageRight;
//    std::vector<double> vTimestamps;
    std::cout << "Loading Images" << std::endl;
    loadImages(kittiPath, m_leftImages, m_rightImages, m_timeStamps);
    this->m_imagesCount = m_leftImages.size();
    this->m_timeTrackingStatistics.resize(this->m_imagesCount);
    std::cout << std::endl << "-------" << std::endl;
    std::cout << "Start processing sequence ..." << std::endl;
    std::cout << "Images in the sequence: " << this->m_imagesCount << std::endl;
}

KittiRunner::~KittiRunner(){
}

void KittiRunner::loadImages(const std::string &path, std::vector<std::string> &vstrImageLeft,
                std::vector<std::string> &vstrImageRight, std::vector<double> &timeStamps)
{
    std::cout << "Parsing times.txt" << std::endl;
    std::string timeStampFilePath = path + "/times.txt";
    std::cout << timeStampFilePath << std::endl;
    std::ifstream inputFileStream(timeStampFilePath.c_str());

    std::string line;
    while (std::getline(inputFileStream, line))
    {
        if(!line.empty()){

            std::istringstream iss(line);
            double timeStamp;
            iss >> timeStamp;
            timeStamps.push_back(timeStamp);
        }
    }

    std::cout << "Parsing times.txt" << std::endl;
    std::cout << "adding images to arrays" << std::endl;
    std::string strPrefixLeft = path + "/image_2/";
    std::string strPrefixRight = path + "/image_3/";

    vstrImageLeft.resize(timeStamps.size());
    vstrImageRight.resize(timeStamps.size());

    for(unsigned long i=500; i<timeStamps.size(); i++)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << i;
        vstrImageLeft[i-500] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i-500] = strPrefixRight + ss.str() + ".png";
    }
}

size_t KittiRunner::GetImagesCount() {
    return this->m_imagesCount;
}

void KittiRunner::ShutDown() {
    // Stop all threads
    this->m_slammer->Shutdown();

    // Tracking time statistics
    std::sort(this->m_timeTrackingStatistics.begin(),this->m_timeTrackingStatistics.end());
    double totaltime = 0;
    for(size_t ni=0; ni<this->m_imagesCount; ni++)
    {
        totaltime+=this->m_timeTrackingStatistics[ni];
    }
    std::cout << "-------" << std::endl << std::endl;
    std::cout << "median tracking time: " << this->m_timeTrackingStatistics[this->m_imagesCount/2] << std::endl;
    std::cout << "mean tracking time: " << totaltime/this->m_imagesCount << std::endl;

    // Save camera trajectory
    //SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");
}

void KittiRunner::ProcessImage(size_t imageNumber) {
    cv::Mat imLeft, imRight;
    cv::Mat R1, R2, P1, P2;
    cv::Rect validRoI[2];

    //CAMERA PARAMETERS
        cv::Mat mtxLeft = (cv::Mat_<double>(3, 3) <<
        669.783, 0, 637.704,
        0, 669.783, 360.875,
        0, 0, 1);
        cv::Mat distLeft = (cv::Mat_<double>(5, 1) << -0.173042, 0.0258831, 0, 0, 0);
        cv::Mat mtxRight = (cv::Mat_<double>(3, 3) <<
        700.225, 0, 660.759,
        0, 700.225, 364.782,
        0, 0, 1);
        cv::Mat distRight = (cv::Mat_<double>(5, 1) << -0.174209, 0.026726, 0, 0, 0);
        cv::Mat rodrigues = (cv::Mat_<double>(3, 1) << -0.0132397, 0.021005, -0.00121284);
        cv::Mat R;
        cv::Rodrigues(rodrigues, R);
        cv::Mat Q;
        cv::Mat T = (cv::Mat_<double>(3, 1) << -0.12, 0, 0);
         cv::Mat rmap[2][2];

        

    // Read left and right images from file
    std::cout << "reading image: " << this->m_leftImages[imageNumber] << std::endl;
    imLeft = cv::imread(this->m_leftImages[imageNumber],CV_LOAD_IMAGE_UNCHANGED);
    int width = imLeft.cols;
    int height = imLeft.rows;
    cv::Size stdSize = cv::Size(width, height);
    if(this->m_isStereo){
        std::cout << "reading image: " << this->m_rightImages[imageNumber] << std::endl;
        imRight = cv::imread(this->m_rightImages[imageNumber],CV_LOAD_IMAGE_UNCHANGED);

          
        cv::stereoRectify(mtxLeft, distLeft, mtxRight, distRight, stdSize, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0.0, stdSize, &validRoI[0], &validRoI[1]);
        cv::initUndistortRectifyMap(mtxLeft, distLeft, R1, P1, stdSize, CV_16SC2, rmap[0][0], rmap[0][1]);
        cv::initUndistortRectifyMap(mtxRight, distRight, R2, P2, stdSize, CV_16SC2, rmap[1][0], rmap[1][1]);
        cv::remap(imLeft, imLeft, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
        cv::remap(imRight, imRight, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);




    }
    //std::cout << "loaded images " << std::endl;
    double tframe = this->m_timeStamps[imageNumber];

    if(imLeft.empty())
    {
        std::cerr << std::endl << "Failed to load image at: "
                  << std::string(this->m_leftImages[imageNumber]) << std::endl;
    }
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    //std::cout << "calling track " << std::endl;
    // Pass the images to the SLAM system
    if(this->m_isStereo){
        m_slammer->Track(imLeft,imRight,tframe);
    }
    else {
        m_slammer->Track(imLeft,tframe);
    }


    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    this->m_timeTrackingStatistics[imageNumber]=ttrack;

    // Wait to load the next frame
    double Ti=0;
    if(imageNumber < this->m_imagesCount-1)
        Ti = this->m_timeStamps[imageNumber+1]-tframe;
    else if(imageNumber>0)
        Ti = tframe-this->m_timeStamps[imageNumber-1];

    if(ttrack<Ti)
        usleep(static_cast<int>((Ti-ttrack)*1e6));

    //std::cout << "Image " << imageNumber << " processed" << std::endl;
}
