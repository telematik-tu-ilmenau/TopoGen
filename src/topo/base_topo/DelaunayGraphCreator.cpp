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

#include "DelaunayGraphCreator.hpp"
#include "geo/CityNode.hpp"
#include "geo/GeometricHelpers.hpp"
#include <vector>
#include <cassert>
#include "geo/TriangulationEdge.hpp"

DelaunayGraphCreator::DelaunayGraphCreator(Locations& cities)
    : _baseTopo(new BaseTopology),
      _graph(_baseTopo->getGraph()),
      _nodeIDmap(new NodeIdMap),
      _nodeGeoNodeMap(_baseTopo->getNodeMap()),
      _sphereLocationMap(new LocationMap),
      _cities(new CityMap),
      _points(new PointVector) {
    using CGALPrimitives::createPoint;

    for (Locations::iterator city = cities.begin(); city != cities.end(); ++city) {
        Point_3 city_loc = createPoint((*city)->lat() + 90.0, (*city)->lon());
        _cities->insert(std::make_pair(city_loc, *city));
        _sphereLocationMap->insert(std::make_pair(*city, city_loc));

        _points->push_back(city_loc);
        Graph::Node u = _baseTopo->addNode(*city);
        (*_nodeIDmap)[(*city)->id()] = u;
    }
}

DelaunayGraphCreator::~DelaunayGraphCreator() {
}

void DelaunayGraphCreator::create(void) {
    std::unique_ptr<Polyhedron_3> _poly(new Polyhedron_3);

    // compute convex hull of non-collinear points
    CGAL::convex_hull_3(_points->begin(), _points->end(), *_poly);

    for (auto it = _poly->edges_begin(); it != _poly->edges_end(); ++it) {
        Point_3 u = it->vertex()->point();
        Point_3 v = it->opposite()->vertex()->point();
        int id_u = (*_cities)[u]->id();
        int id_v = (*_cities)[v]->id();

        Graph::Node n1 = (*_nodeIDmap)[id_u];
        Graph::Node n2 = (*_nodeIDmap)[id_v];
        GeographicEdge_Ptr edge_ptr(new TriangulationEdge);
        _baseTopo->addEdge(n1, n2, edge_ptr);
        assert(id_u != id_v);
    }
}

BaseTopology_Ptr DelaunayGraphCreator::getTopology(void) {
    return _baseTopo;
}
