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

#ifndef BASETOPOLOGY_HPP
#define BASETOPOLOGY_HPP

#include "geo/GeographicEdge.hpp"
#include "geo/GeographicNode.hpp"
#include "topo/Graph.hpp"
#include <memory>
#include <list>
#include <map>

typedef Graph::NodeMap<GeographicNode_Ptr> NodeMap;
typedef std::shared_ptr<NodeMap> NodeMap_Ptr;

typedef std::map<unsigned int, Graph::Node> GeoNodeMap;
typedef std::shared_ptr<GeoNodeMap> GeoNodeMap_Ptr;

typedef Graph::EdgeMap<GeographicEdge_Ptr> EdgeMap;
typedef std::shared_ptr<EdgeMap> EdgeMap_Ptr;

class BaseTopology {
   public:
    BaseTopology();
    Graph::Node addNode(GeographicNode_Ptr& node);
    Graph::Edge addEdge(Graph::Node& u, Graph::Node& v, GeographicEdge_Ptr& e);
    NodeMap_Ptr getNodeMap();
    GeoNodeMap_Ptr getGeoNodeMap();
    EdgeMap_Ptr getEdgeMap();
    Graph_Ptr getGraph();
    void prune();

    // debug
    std::vector<std::pair<double, double>> getHighestDegreeNodes(unsigned int amount = 2, bool USonly = false);

   protected:
   private:
    Graph_Ptr _graph;
    NodeMap_Ptr _nodeGeoNodeMap;
    EdgeMap_Ptr _edgeGeoMap;
    GeoNodeMap_Ptr _geoNodeMap;
};

typedef std::shared_ptr<BaseTopology> BaseTopology_Ptr;

#endif
