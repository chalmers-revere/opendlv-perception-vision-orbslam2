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

#include "orbmap.hpp"

OrbMap::OrbMap() : m_OrbKeyFrameOrigins(), m_keyFrames(), m_mapPoints(), m_referenceMapPoints(), m_maxOrbKeyFrameId(0), m_majorChangeIndex(0)
{
}
OrbMap::~OrbMap()
{
    this->Reset();
}

void OrbMap::PushOrbKeyFrame(std::shared_ptr<OrbKeyFrame> orbKeyFrame) {
    std::lock_guard<std::mutex> lock(this->m_mapMutex);
    this->m_keyFrames.insert(orbKeyFrame);
    if (orbKeyFrame->m_id > this->m_maxOrbKeyFrameId) {
        this->m_maxOrbKeyFrameId = orbKeyFrame->m_id;
    }
}

void OrbMap::PushOrbMapPoint(std::shared_ptr<OrbMapPoint> orbMapPoint)
{
    std::lock_guard<std::mutex> lock(this->m_mapMutex);
    this->m_mapPoints.insert(orbMapPoint);
}

void OrbMap::DeleteOrbMapPoint(std::shared_ptr<OrbMapPoint> orbMapPoint)
{
    std::lock_guard<std::mutex> lock(this->m_mapMutex);
    this->m_mapPoints.erase(orbMapPoint);

}

void OrbMap::DeleteOrbKeyFrame(std::shared_ptr<OrbKeyFrame> orbKeyFrame)
{
    std::lock_guard<std::mutex> lock(this->m_mapMutex);
    this->m_keyFrames.erase(orbKeyFrame);
}

void OrbMap::SetReferenceMapPoints(std::vector<std::shared_ptr<OrbMapPoint>> referenceMapPoints)
{
    std::lock_guard<std::mutex> lock(this->m_mapMutex);
    this->m_referenceMapPoints = referenceMapPoints;
}

std::vector<std::shared_ptr<OrbKeyFrame>> OrbMap::GetAllKeyFrames()
{
    std::lock_guard<std::mutex> lock(this->m_mapMutex);
    return std::vector<std::shared_ptr<OrbKeyFrame>>(m_keyFrames.begin(),m_keyFrames.end());
}

std::vector<std::shared_ptr<OrbMapPoint>> OrbMap::GetAllMapPoints()
{
    std::lock_guard<std::mutex> lock(this->m_mapMutex);
    return std::vector<std::shared_ptr<OrbMapPoint>>(m_mapPoints.begin(),m_mapPoints.end());
}

std::vector<std::shared_ptr<OrbMapPoint>> OrbMap::GetReferenceMapPoints()
{
    std::lock_guard<std::mutex> lock(this->m_mapMutex);
    return this->m_referenceMapPoints;
}

void OrbMap::IncrementMajorChangeIndex()
{
    std::lock_guard<std::mutex> lock(this->m_mapMutex);
    this->m_majorChangeIndex += 1;
}

int OrbMap::LastMajorChangeIndex()
{
    std::lock_guard<std::mutex> lock(this->m_mapMutex);
    return this->m_majorChangeIndex;
}

long unsigned int OrbMap::OrbMapPointsCount()
{
    std::lock_guard<std::mutex> lock(this->m_mapMutex);
    return this->m_mapPoints.size();
}

long unsigned OrbMap::OrbKeyFramesCount()
{
    std::lock_guard<std::mutex> lock(this->m_mapMutex);
    return this->m_keyFrames.size();
}

long unsigned int OrbMap::MaxKeyFrameId()
{
    std::lock_guard<std::mutex> lock(this->m_mapMutex);
    return this->m_maxOrbKeyFrameId;
}

void OrbMap::Reset()
{
    std::lock_guard<std::mutex> lock(this->m_mapMutex);
    this->m_keyFrames.erase(m_keyFrames.begin(), m_keyFrames.end());
    this->m_mapPoints.erase(m_mapPoints.begin(), m_mapPoints.end());
    this->m_referenceMapPoints.erase(m_referenceMapPoints.begin(), m_referenceMapPoints.end());
    this->m_OrbKeyFrameOrigins.erase(m_OrbKeyFrameOrigins.begin(), m_OrbKeyFrameOrigins.end());
    this->m_keyFrames.clear();
    this->m_mapPoints.clear();
    this->m_referenceMapPoints.clear();
    this->m_OrbKeyFrameOrigins.clear();
    this->m_majorChangeIndex = 0;
    this->m_maxOrbKeyFrameId = 0;
}
