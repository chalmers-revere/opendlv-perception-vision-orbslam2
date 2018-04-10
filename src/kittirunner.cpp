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

KittiRunner::KittiRunner(const std::string &kittiPath,std::shared_ptr<Selflocalization> slammer){
    std::vector<std::string> vstrImageLeft;
    std::vector<std::string> vstrImageRight;
    std::vector<double> vTimestamps;
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
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            std::cerr << std::endl << "Failed to load image at: "
                 << std::string(vstrImageLeft[ni]) << std::endl;
        }
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the images to the SLAM system
        slammer->Track(imLeft,imRight,tframe);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

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
                std::vector<std::string> &vstrImageRight, std::vector<double> &vTimestamps)
{
    std::ifstream fTimes;
    std::string strPathTimeFile = path + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        std::string s;
        std::getline(fTimes,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    std::string strPrefixLeft = path + "/image_0/";
    std::string strPrefixRight = path + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}