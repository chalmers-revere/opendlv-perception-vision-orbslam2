/**
 * File: OrbFeatureVector.h
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: feature vector
 * Adapted for OpenDLV 2018
 *
 */


#ifndef ORBFEATUREVECTOR_HPP
#define ORBFEATUREVECTOR_HPP


#include "orbbowvector.hpp"
#include <map>
#include <vector>
#include <iostream>


/// Vector of nodes with indexes of local features
class OrbFeatureVector: 
  public std::map<uint32_t, std::vector<uint32_t> >
{
public:

  OrbFeatureVector();
  
  ~OrbFeatureVector();
  
  /**
   * Adds a feature to an existing node, or adds a new node with an initial
   * feature
   * @param id node id to add or to modify
   * @param i_feature index of feature to add to the given node
   */
  void addFeature(uint32_t id, uint32_t i_feature);

  /**
   * Sends a string versions of the feature vector through the stream
   * @param out stream
   * @param v feature vector
   */
  friend std::ostream& operator<<(std::ostream &out, const OrbFeatureVector &featureVector);
    
};

#endif
