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

#include "GraphOutput.hpp"
#include "geo/GeographicNode.hpp"
#include "geo/SeaCableLandingPoint.hpp"
#include "geo/CityNode.hpp"
#include "geo/SeaCableNode.hpp"
#include "geo/SeaCableEdge.hpp"

GraphOutput::GraphOutput(BaseTopology_Ptr baseTopo, std::ofstream& nodeFile, std::ofstream& edgeFile)
    : _baseTopo(baseTopo),
      _graph(_baseTopo->getGraph()),
      _nodeToCity(_baseTopo->getNodeMap()),
      _nodeFile(nodeFile),
      _edgeFile(edgeFile) {
}

void GraphOutput::writeNodes(void) {
    for (Graph::NodeIt n(*_graph); n != lemon::INVALID; ++n) {
        GeographicNode_Ptr node = (*_nodeToCity)[n];
        if (!node->isValid())
            continue;

        _nodeFile << _graph->id(n) << "\t";

        if (dynamic_cast<CityNode*>(node.get()))
            _nodeFile << dynamic_cast<CityNode*>(node.get())->name();

        if (dynamic_cast<SeaCableLandingPoint*>(node.get()))
            _nodeFile << dynamic_cast<SeaCableLandingPoint*>(node.get())->name();

        if (dynamic_cast<SeaCableNode*>(node.get()))
            _nodeFile << "Seacable Waypoint";

        _nodeFile << "\t" << (*_nodeToCity)[n]->lat() << "\t" << (*_nodeToCity)[n]->lon() << std::endl;
    }
}

void GraphOutput::writeEdges(void) {
    // write out edges

    auto _nodeInfos = _baseTopo->getNodeMap();
    for (Graph::EdgeIt edge(*_graph); edge != lemon::INVALID; ++edge) {
        GeographicNode_Ptr n1 = (*_nodeInfos)[_graph->u(edge)];
        GeographicNode_Ptr n2 = (*_nodeInfos)[_graph->v(edge)];
        if (!n1->isValid() || !n2->isValid())
            continue;

        bool seacable = false;
        EdgeMap_Ptr edgeMap = _baseTopo->getEdgeMap();
        GeographicEdge_Ptr edge_Ptr = (*edgeMap)[edge];

        if (dynamic_cast<SeaCableEdge*>(edge_Ptr.get()))
            seacable = true;

        _edgeFile << _graph->id(_graph->u(Graph::Edge(edge))) << "\t" << _graph->id(_graph->v(Graph::Edge(edge)));

        if (seacable)
            _edgeFile << "\tseacable";
        else
            _edgeFile << "\tnormal";

        _edgeFile << std::endl;
    }
}