/*
 * Copyright (c) 2013-2014, Michael Grey and Markus Theil
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "CityNode.hpp"

CityNode::CityNode(int id,
                   std::string name,
                   double lat,
                   double lon,
                   double population,
                   std::string country,
                   std::string continent)
    : GeographicNode(id, lat, lon),
      _name(name),
      _population(population),
      _country(country),
      _continent(continent),
      _seacableLandingPoint(false) {
}

std::string CityNode::name(void) {
    return _name;
}

CityNode::CityNode(const CityNode& other)
    : GeographicNode(other._id, other._latitude, other._longitude),
      _name(other._name),
      _population(other._population),
      _country(other._country),
      _continent(other._continent),
      _seacableLandingPoint(other._seacableLandingPoint) {
}

CityNode& CityNode::operator=(const CityNode& other) {
    this->_id = other._id;
    this->_name = other._name;
    this->_latitude = other._latitude;
    this->_longitude = other._longitude;
    this->_population = other._population;
    this->_country = other._country;
    this->_continent = other._continent;
    this->_seacableLandingPoint = other._seacableLandingPoint;
    return *this;
}

double CityNode::population() {
    return _population;
}

std::string CityNode::country() {
    return _country;
}

std::string CityNode::continent() {
    return _continent;
}

bool CityNode::operator<(const CityNode& other) const {
    return this->_name.compare(other._name) < 0;
}

bool CityNode::isSeaCableLandingPoint() {
    return _seacableLandingPoint;
}

void CityNode::setSeaCableLandingPoint() {
    _seacableLandingPoint = true;
}
