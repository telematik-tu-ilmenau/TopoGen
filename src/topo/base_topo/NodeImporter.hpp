/*
 * Copyright (c) 2013-2015, Michael Grey and Markus Theil
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

#ifndef NODEIMPORTER_HPP
#define NODEIMPORTER_HPP

#include "geo/CityNode.hpp"
#include "geo/GeographicNode.hpp"
#include "geo/GeographicPosition.hpp"
#include "geo/SeaCableLandingPoint.hpp"
#include "geo/SeaCableNode.hpp"
#include "topo/base_topo/BaseTopology.hpp"
#include <memory>

class NodeImporter {
   public:
    NodeImporter(void);

    void importCitiesFromFile(void);
    void importCities(const std::string& seed);

    void importSeacableLandingPoints();
    void importSubmarineCableEdgesWaypoints();

    Locations_Ptr getLocations(void);

    void addNode(GeographicNode_Ptr node);

    void importSubmarineCableEdges(BaseTopology_Ptr base_topo);

   protected:
   private:
    GeographicNode_Ptr findNearest(GeographicNode_Ptr& node);

    int _nodenumber;

    Locations_Ptr _locations;

    std::string _dbFilename;
    static double constexpr DIST_TRESHOLD = 0.0005;

    std::map<GeographicPositionTuple, GeographicPositionTuple>
        _fallbackProjection;  // quick hack: ensure correct placement of seacable nodes
};

#endif  // NODEIMPORTER_HPP
