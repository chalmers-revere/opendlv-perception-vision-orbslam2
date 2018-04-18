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
    std::string strPrefixLeft = path + "/image_0/";
    std::string strPrefixRight = path + "/image_1/";

    vstrImageLeft.resize(timeStamps.size());
    vstrImageRight.resize(timeStamps.size());

    for(unsigned long i=0; i<timeStamps.size(); i++)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
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
    // Read left and right images from file
    std::cout << "reading image: " << this->m_leftImages[imageNumber] << std::endl;
    imLeft = cv::imread(this->m_leftImages[imageNumber],CV_LOAD_IMAGE_UNCHANGED);
    if(this->m_isStereo){
        std::cout << "reading image: " << this->m_rightImages[imageNumber] << std::endl;
        imRight = cv::imread(this->m_rightImages[imageNumber],CV_LOAD_IMAGE_UNCHANGED);
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
    double T=0;
    if(imageNumber < this->m_imagesCount-1)
        T = this->m_timeStamps[imageNumber+1]-tframe;
    else if(imageNumber>0)
        T = tframe-this->m_timeStamps[imageNumber-1];

    if(ttrack<T)
        usleep(static_cast<int>((T-ttrack)*1e6));

    std::cout << "Image " << imageNumber << " processed" << std::endl;
}
