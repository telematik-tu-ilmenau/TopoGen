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

#include "SimulationTopology.hpp"
#include "geo/SimulationEdge.hpp"
#include "geo/CityNode.hpp"
#include <lemon/dijkstra.h>
#include <lemon/path.h>
#include "geo/GeometricHelpers.hpp"
#include <cassert>

SimulationTopology::SimulationTopology(BaseTopology_Ptr& baseTopo) : _baseTopo(baseTopo) {
}

GeographicNode_Ptr SimulationTopology::findNearest(GeographicNode_Ptr& node) {
    GeographicNode_Ptr nearest = nullptr;
    double currentDist = std::numeric_limits<double>::max();

    NodeMap_Ptr nodeMap = _baseTopo->getNodeMap();
    for (Graph::NodeIt it(*_baseTopo->getGraph()); it != lemon::INVALID; ++it) {
        GeographicNode_Ptr otherNode = (*nodeMap)[it];

        CityNode* cityNode = dynamic_cast<CityNode*>(otherNode.get());
        if (cityNode != nullptr) {
            double dist = GeometricHelpers::sphericalDist(node, otherNode);
            if (dist < currentDist) {
                nearest = otherNode;
                currentDist = dist;
            }
        }
    }

    return nearest;
}

void SimulationTopology::addNode(SimulationNode_Ptr node) {
    // get nearest node
    NodeMap_Ptr nodeMap = _baseTopo->getNodeMap();

    GeographicNode_Ptr castedNode = std::static_pointer_cast<GeographicNode>(node);
    GeographicNode_Ptr nearestNode = findNearest(castedNode);
    assert(static_cast<bool>(nearestNode));
    int id = nearestNode->id();

    // add node to topo
    Graph::Node newNode = _baseTopo->addNode(castedNode);
    int newId = _baseTopo->getGraph()->id(newNode);
    node->setId(newId);

    Graph::Node otherNode = _baseTopo->getGraph()->nodeFromId(id);
    SimulationEdge_Ptr simEdge(new SimulationEdge);
    GeographicEdge_Ptr castedEdge = std::static_pointer_cast<GeographicEdge>(simEdge);
    _baseTopo->addEdge(newNode, otherNode, castedEdge);
}

std::pair<Locations_Ptr, double> SimulationTopology::pathBetweenNodes(SimulationNode_Ptr n1, SimulationNode_Ptr n2) {
    auto graph = _baseTopo->getGraph();

    // make graph nodes
    Graph::Node u = graph->nodeFromId(n1->id());
    Graph::Node v = graph->nodeFromId(n2->id());

    // set lengths for all edges
    Graph::EdgeMap<double> lengthMap(*_baseTopo->getGraph());
    auto _nodeInfos = _baseTopo->getNodeMap();
    for (Graph::EdgeIt edge(*graph); edge != lemon::INVALID; ++edge) {
        GeographicNode_Ptr n1 = (*_nodeInfos)[graph->u(edge)];
        GeographicNode_Ptr n2 = (*_nodeInfos)[graph->v(edge)];
        lengthMap[edge] = GeometricHelpers::sphericalDist(n1, n2);
    }

    typedef lemon::Path<Graph> GraphPath;
    GraphPath path;
    double dist;

    bool reached = dijkstra(*graph, lengthMap).path(path).dist(dist).run(u, v);
    assert(reached);

    Locations_Ptr ret(new Locations);

    // append path nodes to path
    typedef lemon::PathNodeIt<GraphPath> PathNodeIt;
    for (PathNodeIt it(*graph, path); it != lemon::INVALID; ++it) {
        Graph::Node u = PathNodeIt::Node(it);
        GeographicNode_Ptr node = (*_nodeInfos)[u];
        ret->push_back(node);
    }

    // return stuff
    return std::make_pair(ret, dist);
}
