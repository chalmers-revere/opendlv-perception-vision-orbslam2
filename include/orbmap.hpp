/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* Modified for use within the OpenDLV framework by Marcus Andersson, Martin Baerveldt, Linus Eiderström Swahn and Pontus Pohl
* Copyright (C) 2018 Chalmers Revere
* For more information see <https://github.com/chalmers-revere/opendlv-perception-vision-orbslam2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBMAP_HPP
#define ORBMAP_HPP


#include <mutex>
#include <algorithm>
#include <memory>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <vector>
#include <list>
#include <utility>
#include <orbkeyframe.hpp>

class OrbKeyFrame;
class OrbMapPoint;


class OrbMap {
 public:
    /**
     *  Çonstructor
     */
    OrbMap();
    ~OrbMap();
    /**
     * PushOrbKeyFrame - Push Keyframe to Map.
     */
    void PushOrbKeyFrame(std::shared_ptr<OrbKeyFrame> keyFrame);
    /**
     * PushOrbMapPoint - Push OrbMapPoint to Map.
     */
    void PushOrbMapPoint(std::shared_ptr<OrbMapPoint> orbMapPoint);
    /**
     * DeleteOrbMapPoint - Deletes OrbMapPoint from if it exists in Map.
     */
    void DeleteOrbMapPoint(std::shared_ptr<OrbMapPoint> orbMapPoint);
    /**
     * DeleteOrbKeyFrame - Deletes OrbKeyFrame from if it exists in Map.
     */
    void DeleteOrbKeyFrame(std::shared_ptr<OrbKeyFrame> orbKeyFrame);
    /**
     * SetReferenceMapPoints - Set reference Map Points in Map.
     */
    void SetReferenceMapPoints(std::vector<std::shared_ptr<OrbMapPoint>> referenceMapPoints);
    /**
     * GetAllKeyFrames - Returns all OrbKeyFrames
     */
    std::vector<std::shared_ptr<OrbKeyFrame>> GetAllKeyFrames();
    /**
     * GetAllMapPoints - Returns all OrbMapPoints
     */
    std::vector<std::shared_ptr<OrbMapPoint>> GetAllMapPoints();
    /**
     * GetReferenceMapPoints - Returns all ReferenceMapPoints
     */
    std::vector<std::shared_ptr<OrbMapPoint>> GetReferenceMapPoints();
    void IncrementMajorChangeIndex();
    int LastMajorChangeIndex();
    /**
     *  OrbMapPointsCount - return total number of OrbMapPoints in Map.
     */
    long unsigned int OrbMapPointsCount();
    /**
     *  OrbKeyFramesCount - return total number of OrbKeyFrames in Map.
     */
    long unsigned  OrbKeyFramesCount();
    /**
     * MaxKeyFrameId - Return the maximum OrbKeyFrame Id.
     */
    long unsigned int MaxKeyFrameId();
    /**
     * Reset - Resets the Map.
     */
    void Reset();

    std::vector<std::shared_ptr<OrbKeyFrame>> GetKeyFrameOrigins(){ return m_OrbKeyFrameOrigins; };

    std::mutex m_MapUpdateMutex = {};
    std::vector<std::shared_ptr<OrbKeyFrame>> m_OrbKeyFrameOrigins;

 private:
    std::set<std::shared_ptr<OrbKeyFrame>> m_keyFrames;
    std::set<std::shared_ptr<OrbMapPoint>> m_mapPoints;
    std::vector<std::shared_ptr<OrbMapPoint>> m_referenceMapPoints;
    long unsigned int m_maxOrbKeyFrameId;
    int m_majorChangeIndex;
    
    std::mutex m_mapMutex = {};
    std::mutex m_orbMapPointCreationMutex = {};
};

#endif //ORBMAP_HPP