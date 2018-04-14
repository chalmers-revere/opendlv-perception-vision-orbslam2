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

KittiRunner::KittiRunner(const std::string &kittiPath,bool isStereo,std::shared_ptr<Selflocalization> slammer){
    std::vector<std::string> vstrImageLeft;
    std::vector<std::string> vstrImageRight;
    std::vector<double> vTimestamps;
    std::cout << "Loading Images" << std::endl;
    loadImages(kittiPath, vstrImageLeft, vstrImageRight, vTimestamps);
    const int nImages = vstrImageLeft.size();
    std::vector<double> vTimesTrack;
    vTimesTrack.resize(nImages);

    std::cout << std::endl << "-------" << std::endl;
    std::cout << "Start processing sequence ..." << std::endl;
    std::cout << "Images in the sequence: " << nImages << std::endl;   

    // Main loop
    cv::Mat imLeft, imRight;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        std::cout << "reading image: " << vstrImageLeft[ni] << std::endl;
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        if(isStereo){
            imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
        }
        std::cout << "loaded images " << std::endl;
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            std::cerr << std::endl << "Failed to load image at: "
                 << std::string(vstrImageLeft[ni]) << std::endl;
        }
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        std::cout << "calling track " << std::endl;
        // Pass the images to the SLAM system
        if(isStereo){
            slammer->Track(imLeft,imRight,tframe);    
        }
        slammer->Track(imLeft,tframe);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::cout << "calculating sleep " << std::endl;
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep(static_cast<int>((T-ttrack)*1e6));
    }

    // Stop all threads
    slammer->Shutdown();

    // Tracking time statistics
    std::sort(vTimesTrack.begin(),vTimesTrack.end());
    double totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    std::cout << "-------" << std::endl << std::endl;
    std::cout << "median tracking time: " << vTimesTrack[nImages/2] << std::endl;
    std::cout << "mean tracking time: " << totaltime/nImages << std::endl;

    // Save camera trajectory
    //SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");
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

        std::cout << line << std::endl;
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

    for(unsigned long i=0; i<timeStamps.size(); i++)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << i;
        std::cout << "adding image" << std::endl;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}