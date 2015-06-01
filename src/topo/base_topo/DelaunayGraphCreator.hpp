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

#ifndef DELAUNAYGRAPHCREATOR_HPP
#define DELAUNAYGRAPHCREATOR_HPP

#include "BaseTopology.hpp"
#include "CGALPrimitives.hpp"
#include "topo/Graph.hpp"
#include <CGAL/algorithm.h>
#include <CGAL/convex_hull_3.h>
#include <memory>

class DelaunayGraphCreator {
   public:
    DelaunayGraphCreator(Locations& cities);
    DelaunayGraphCreator(const DelaunayGraphCreator& orig);
    virtual ~DelaunayGraphCreator();

    void create(void);
    BaseTopology_Ptr getTopology(void);

   private:
    BaseTopology_Ptr _baseTopo;
    Graph_Ptr _graph;

    typedef std::map<int, Graph::Node> NodeIdMap;
    typedef std::shared_ptr<NodeIdMap> NodeIdMap_Ptr;
    NodeIdMap_Ptr _nodeIDmap;

    NodeMap_Ptr _nodeGeoNodeMap;

    typedef std::map<GeographicNode_Ptr, Point_3> LocationMap;
    typedef std::shared_ptr<LocationMap> LocationMap_Ptr;
    LocationMap_Ptr _sphereLocationMap;

    CityMap_Ptr _cities;

    typedef std::vector<Point_3> PointVector;
    typedef std::shared_ptr<PointVector> PointVector_Ptr;
    PointVector_Ptr _points;
};

#endif
