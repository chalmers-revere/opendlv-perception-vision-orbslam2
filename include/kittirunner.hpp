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

#ifndef KITTIRUNNER_HPP
#define KITTIRUNNER_HPP

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>
#include<selflocalization.hpp>

class KittiRunner
{
public:
    KittiRunner(const std::string &path, bool isStereo, std::shared_ptr<Selflocalization> slammer);
    KittiRunner(KittiRunner const &) = delete;
    KittiRunner &operator=(KittiRunner const &) = delete;
    ~KittiRunner();
    void ProcessImage(size_t imageNumber);
    void ShutDown();
    size_t GetImagesCount();

private:
    void loadImages(const std::string &path, std::vector<std::string> &vstrImageLeft,
                std::vector<std::string> &vstrImageRight, std::vector<double> &vTimestamps);
    size_t m_imagesCount;
    bool m_isStereo;
    std::shared_ptr<Selflocalization> m_slammer;
    std::vector<std::string> m_leftImages;
    std::vector<std::string> m_rightImages;
    std::vector<double> m_timeStamps;
    std::vector<double> m_timeTrackingStatistics;

    //Rectification parameters
        cv::Mat mtxLeft = {}; 
        cv::Mat distLeft = {};
        cv::Mat mtxRight = {};
        cv::Mat distRight = {};
        cv::Mat R = {};
        cv::Mat rodrigues = {};
        cv::Mat Q = {};
        cv::Mat T = {};
        cv::Mat rmap[2][2];
        cv::Mat imgL = {};
        cv::Mat imgR = {};
        cv::Size stdSize = {}; 

        cv::Mat R1 = {};
        cv::Mat R2 = {};
        cv::Mat P1 = {};
        cv::Mat P2 = {};
        cv::Rect validRoI[2] = {};
};
#endif