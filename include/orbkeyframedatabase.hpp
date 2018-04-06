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

#ifndef OPENDLV_ORBKEYFRAMEDATABASE_HPP
#define OPENDLV_ORBKEYFRAMEDATABASE_HPP

#include "orbkeyframe.hpp"
#include "orbframe.hpp"
#include "orbvocabulary.hpp"
#include <vector>
#include <list>

class OrbKeyFrame;
class OrbFrame;
class OrbVocabulary;

class OrbKeyFrameDatabase
{
public:

    OrbKeyFrameDatabase(OrbVocabulary &voc);

    void add(std::shared_ptr<OrbKeyFrame> pKF);

    void erase(std::shared_ptr<OrbKeyFrame> pKF);

    void clear();

    // Loop Detection
    std::vector<std::shared_ptr<OrbKeyFrame>> DetectLoopCandidates(std::shared_ptr<OrbKeyFrame> pKF, float minScore);

    // Relocalization
    std::vector<std::shared_ptr<OrbKeyFrame>> DetectRelocalizationCandidates(std::shared_ptr<OrbFrame> F);

protected:

    // Associated vocabulary
    std::shared_ptr<OrbVocabulary> mpVoc;

    // Inverted file
    std::vector<std::list<std::shared_ptr<OrbKeyFrame>>> mvInvertedFile = {};

    // Mutex
    std::mutex mMutex = {};
};

#endif //OPENDLV_ORBKEYFRAMEDATABASE_HPP
