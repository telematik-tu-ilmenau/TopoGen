/*
 * Copyright (c) 2015, Michael Grey and Markus Theil
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

#include "JSONOutput.hpp"

#include "geo/CityNode.hpp"
#include "geo/GeographicNode.hpp"
#include "geo/GeometricHelpers.hpp"
#include "geo/SeaCableEdge.hpp"
#include "geo/SeaCableLandingPoint.hpp"
#include "geo/SeaCableNode.hpp"
#include "geo/SimulationNode.hpp"
#include "geo/SimulationEdge.hpp"
#include <fstream>

JSONOutput::JSONOutput(BaseTopology_Ptr baseTopo)
    : _baseTopo(baseTopo), _graph(_baseTopo->getGraph()), _nodeToCity(_baseTopo->getNodeMap()) {
}

JSONOutput::~JSONOutput() {
}

void JSONOutput::createJSON(void) {
    for (Graph::NodeIt n(*_graph); n != lemon::INVALID; ++n) {
        GeographicNode_Ptr node = (*_nodeToCity)[n];
        if (!node->isValid())
            continue;

        Json::Value nodeJSON;
        nodeJSON["id"] = _graph->id(n);

        if (dynamic_cast<CityNode*>(node.get())) {
            nodeJSON["type"] = "City";
            nodeJSON["name"] = dynamic_cast<CityNode*>(node.get())->name();
        }

        if (dynamic_cast<SeaCableLandingPoint*>(node.get())) {
            nodeJSON["type"] = "Seacable Landing Point";
            nodeJSON["name"] = dynamic_cast<SeaCableLandingPoint*>(node.get())->name();
        }

        if (dynamic_cast<SeaCableNode*>(node.get())) {
            nodeJSON["type"] = "Seacable Waypoint";
            nodeJSON["name"] = "Seacable Waypoint";
        }

        if (dynamic_cast<SimulationNode*>(node.get())) {
            auto simNode = dynamic_cast<SimulationNode*>(node.get());
            nodeJSON["type"] = "Simulation Node";
            nodeJSON["name"] = "Simulation Node";
            nodeJSON["id"] = simNode->id();
            nodeJSON["outer_id"] = simNode->outerID();
        }

        nodeJSON["latitude"] = (*_nodeToCity)[n]->lat();
        nodeJSON["longitude"] = (*_nodeToCity)[n]->lon();
        _json["nodes"].append(nodeJSON);
    }

    // write out edges

    auto _nodeInfos = _baseTopo->getNodeMap();
    for (Graph::EdgeIt edge(*_graph); edge != lemon::INVALID; ++edge) {
        GeographicNode_Ptr n1 = (*_nodeInfos)[_graph->u(edge)];
        GeographicNode_Ptr n2 = (*_nodeInfos)[_graph->v(edge)];
        if (!n1->isValid() || !n2->isValid())
            continue;

        Json::Value edgeJSON;

        enum EdgeType { NORMAL_EDGE, SEACABLE_EDGE, SIMULATION_EDGE, UNDEFINED_EDGE };

        EdgeType edgeType = EdgeType::UNDEFINED_EDGE;

        EdgeMap_Ptr edgeMap = _baseTopo->getEdgeMap();
        GeographicEdge_Ptr edge_Ptr = (*edgeMap)[edge];

        if (dynamic_cast<SeaCableEdge*>(edge_Ptr.get())) {
            edgeType = EdgeType::SEACABLE_EDGE;
        } else if (dynamic_cast<SimulationEdge*>(edge_Ptr.get())) {
            edgeType = EdgeType::SIMULATION_EDGE;
        } else {
            edgeType = EdgeType::NORMAL_EDGE;
        }

        edgeJSON["u"] = _graph->id(_graph->u(Graph::Edge(edge)));
        edgeJSON["v"] = _graph->id(_graph->v(Graph::Edge(edge)));

        switch (edgeType) {
            case EdgeType::NORMAL_EDGE:
                edgeJSON["type"] = "normal";
                break;
            case EdgeType::SEACABLE_EDGE:
                edgeJSON["type"] = "seacable";
                break;
            case EdgeType::SIMULATION_EDGE:
                edgeJSON["type"] = "simulation";
                break;
            case EdgeType::UNDEFINED_EDGE:
                edgeJSON["type"] = "undefined";
                break;
        }

        edgeJSON["distance"] = GeometricHelpers::sphericalDistToKM(GeometricHelpers::sphericalDist(n1, n2));

        _json["edges"].append(edgeJSON);
    }
}

void JSONOutput::writePretty(const char* filename) {
    Json::StyledWriter writer;
    std::ofstream graph(filename);
    graph << writer.write(_json);
}

void JSONOutput::write(const char* filename) {
    Json::FastWriter writer;
    std::ofstream graph(filename);
    graph << writer.write(_json);
}
