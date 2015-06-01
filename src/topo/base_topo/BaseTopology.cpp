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

#include "BaseTopology.hpp"
#include "geo/CityNode.hpp"
#include "lemon/maps.h"
#include "lemon/connectivity.h"
#include <boost/log/trivial.hpp>

BaseTopology::BaseTopology()
    : _graph(new Graph),
      _nodeGeoNodeMap(new NodeMap(*_graph)),
      _edgeGeoMap(new EdgeMap(*_graph)),
      _geoNodeMap(new GeoNodeMap) {
}

Graph::Node BaseTopology::addNode(GeographicNode_Ptr& gNode) {
    Graph::Node nd = _graph->addNode();
    (*_nodeGeoNodeMap)[nd] = gNode;
    (*_geoNodeMap)[gNode->id()] = nd;
    return nd;
}

NodeMap_Ptr BaseTopology::getNodeMap() {
    return _nodeGeoNodeMap;
}

Graph_Ptr BaseTopology::getGraph() {
    return _graph;
}

GeoNodeMap_Ptr BaseTopology::getGeoNodeMap() {
    return _geoNodeMap;
}

Graph::Edge BaseTopology::addEdge(Graph::Node& u, Graph::Node& v, GeographicEdge_Ptr& e) {
    Graph::Edge edge = _graph->addEdge(u, v);
    (*_edgeGeoMap)[edge] = e;
    return edge;
}

EdgeMap_Ptr BaseTopology::getEdgeMap() {
    return _edgeGeoMap;
}

void BaseTopology::prune() {
    // extract the biggest connected component and prune the rest
    Graph_Ptr graph = getGraph();
    Graph::NodeMap<int> nodeMap(*graph);
    int numConnectedComponents = lemon::connectedComponents(*graph, nodeMap);
    BOOST_LOG_TRIVIAL(info) << "Graph has " << numConnectedComponents << " components!";

    // count members of first component
    std::map<int, int> componentCounter;
    for (Graph::NodeIt it(*graph); it != lemon::INVALID; ++it) {
        componentCounter[nodeMap[it]]++;
    }

    int maxComponent = 0;
    int maxNum = 0;
    for (auto p : componentCounter) {
        if (p.second > maxNum) {
            maxNum = p.second;
            maxComponent = p.first;
        }
    }

    std::list<int> nodesToDelete;
    for (Graph::NodeIt it(*graph); it != lemon::INVALID; ++it) {
        if (nodeMap[it] != maxComponent)
            nodesToDelete.push_back(graph->id(it));
    }

    for (auto id : nodesToDelete) {
        graph->erase(graph->nodeFromId(id));
        _geoNodeMap->erase(id);
    }
}

std::vector<std::pair<double, double>> BaseTopology::getHighestDegreeNodes(unsigned int amount, bool USonly) {
    std::multimap<int, std::pair<double, double>> degreeMap;
    std::vector<std::pair<double, double>> toReturn;
    Graph_Ptr graph = getGraph();
    Graph::NodeMap<int> nodeMap(*graph);

    for (Graph::NodeIt it(*graph); it != lemon::INVALID; ++it) {
        Graph::Node nd(it);
        int arcs = lemon::countOutArcs(*graph, nd);
        GeographicNode_Ptr& gNode = (*_nodeGeoNodeMap)[nd];
        if (dynamic_cast<CityNode*>(gNode.get()) != NULL) {
            if (USonly == false || static_cast<CityNode*>(gNode.get())->country() == "United States") {
                degreeMap.insert(std::make_pair(arcs, gNode->coord()));
            }
        }
    }
    for (unsigned int i = 0; i < amount; ++i) {
        toReturn.push_back(degreeMap.rbegin()->second);
        degreeMap.erase(std::prev(degreeMap.end()));
    }
    return toReturn;
}
