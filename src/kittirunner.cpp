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

KittiRunner::KittiRunner(const std::string &kittiPath,bool isStereo,std::shared_ptr<Selflocalization> slammer, cv::Mat a_rMap[2][2]):
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
    /*m_rmap[0][0] = a_rMap[0][0];
    m_rmap[0][1] = a_rMap[0][1];
    m_rmap[1][0] = a_rMap[1][0];
    m_rmap[1][1] = a_rMap[1][1];*/

    a_rMap[0][0].copyTo(m_rmap[0][0]);

    a_rMap[0][1].copyTo(m_rmap[0][1]);

    a_rMap[1][0].copyTo(m_rmap[1][0]);

    a_rMap[1][1].copyTo(m_rmap[1][1]);
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
    int offset = 0;
    for(unsigned long i=offset; i<timeStamps.size(); i++)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << i;
        vstrImageLeft[i-offset] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i-offset] = strPrefixRight + ss.str() + ".png";
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
}

void KittiRunner::ProcessImage(size_t imageNumber,float resizeScale) {
    cv::Mat imLeft;
    cv::Mat imRight;

    //CAMERA PARAMETE  

    // Read left and right images from file
    std::cout << "reading image: " << this->m_leftImages[imageNumber] << std::endl;
    cv::Mat imgL = cv::imread(this->m_leftImages[imageNumber],CV_LOAD_IMAGE_UNCHANGED);
    if(imgL.empty())
    {
        std::cerr << std::endl << "Failed to load image at: "
                  << std::string(this->m_leftImages[imageNumber]) << std::endl;
                  return;
    }
    
    if(this->m_isStereo){
        std::cout << "reading image: " << this->m_rightImages[imageNumber] << std::endl;
        cv::Mat imgR = cv::imread(this->m_rightImages[imageNumber],CV_LOAD_IMAGE_UNCHANGED);
        if(m_rmap[0][0].cols==0)
        {
            imLeft=imgL;
            imRight=imgR;
        }
        else
        {
            if(resizeScale < 1)
            {
                cv::resize(imgL, imgL, cv::Size(static_cast<int>(imgL.cols*resizeScale),static_cast<int>(imgL.rows*resizeScale)));
                cv::resize(imgR, imgR, cv::Size(static_cast<int>(imgR.cols*resizeScale),static_cast<int>(imgR.rows*resizeScale)));
            }
            cv::remap(imgL,imLeft, m_rmap[0][0], m_rmap[0][1], cv::INTER_LINEAR);
            cv::remap(imgR,imRight, m_rmap[1][0], m_rmap[1][1], cv::INTER_LINEAR);
        }
        /*int wL = imLeft.cols;
        int hL = imLeft.rows;

        int wR = imRight.cols;
        int hR = imRight.rows;

        std::cout << "Size Left: " << wL << " : " << hL << std::endl;

        std::cout << "Size Right: " << wR << " : " << hR << std::endl;
            cv::namedWindow( "Display window 1", cv::WINDOW_AUTOSIZE );// Create a window for display.
            cv::imshow( "Display window 1", imLeft );   

            cv::namedWindow( "Display window 2", cv::WINDOW_AUTOSIZE );// Create a window for display.
            cv::imshow( "Display window 2", imLeft );   
            cv::waitKey(0);*/

    }else{

        if(resizeScale < 1){
            cv::resize(imgL, imgL, cv::Size(static_cast<int>(imgL.cols*resizeScale),static_cast<int>(imgL.rows*resizeScale)));
        }
    }
    //std::cout << "loaded images " << std::endl;
    double tframe = this->m_timeStamps[imageNumber];
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    std::cout << "calling track " << std::endl;
    // Pass the images to the SLAM system
    if(this->m_isStereo){
        m_slammer->Track(imLeft,imRight,tframe);
    }
    else {
        m_slammer->Track(imgL,tframe);
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
