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
/*
#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp" //this is probably not needed here
*/
#include <iostream>
#include <map>
#include <vector>
#include <fstream>
#include <algorithm>
#include <cmath>

//#include "opendavinci/odcore/base/KeyValueConfiguration.h"


#ifndef ORBBOWVECTOR_HPP
#define ORBBOWVECTOR_HPP


class OrbBowVector:public std::map<uint32_t, double>
 {

 public:
  OrbBowVector();
  ~OrbBowVector();

  //------------------------

	void addWeight(uint32_t id, double v);
	
	void addIfNotExist(uint32_t id, double v);

	void normalize();

	friend std::ostream& operator<<(std::ostream &out, const OrbBowVector &v);

};


#endif //ORBBOWVECTOR_HPP