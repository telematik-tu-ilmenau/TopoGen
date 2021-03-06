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

#include "BetaSkeletonFilter.hpp"

#include "config/Config.hpp"
#include "config/PredefinedValues.hpp"
#include "db/InternetUsageStatistics.hpp"
#include "geo/CityNode.hpp"
#include "geo/GeographicNode.hpp"
#include "geo/GeometricHelpers.hpp"
#include "geo/SeaCableLandingPoint.hpp"
#include "geo/SeaCableNode.hpp"
#include "topo/Graph.hpp"
#include "util/Util.hpp"
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <lemon/bfs.h>
#include <lemon/core.h>
#include <list>
#include <set>

using GeometricHelpers::deg2rad;
using GeometricHelpers::rad2deg;
using GeometricHelpers::sphericalDist;

BetaSkeletonFilter::BetaSkeletonFilter(BaseTopology_Ptr baseTopo)
    : _baseTopo(baseTopo), _graph(_baseTopo->getGraph()), _nodeGeoNodeMap(_baseTopo->getNodeMap()) {
}

BetaSkeletonFilter::~BetaSkeletonFilter() {
}

void BetaSkeletonFilter::generalGabrielFilter() {
    using namespace lemon;
    EdgeList edges_to_delete;

    for (Graph::EdgeIt edge(*_graph); edge != lemon::INVALID; ++edge) {
        if (edge != INVALID && !isBetaSkeletonEdgeGreaterEqualThanOne(edge, 1.0))
            edges_to_delete.push_back(edge);
    }

    for (EdgeList::iterator edge = edges_to_delete.begin(); edge != edges_to_delete.end(); ++edge)
        _graph->erase(*edge);
}

void BetaSkeletonFilter::perCountryBetaFilter() {
    using namespace lemon;

    // create list of cities per country
    typedef std::pair<Graph::Node, CityNode*> Node;
    std::map<std::string, std::vector<Node>> countries;

    for (ListGraph::NodeIt it(*_graph); it != INVALID; ++it) {
        Graph::Node nd(it);
        GeographicNode_Ptr n1 = (*_nodeGeoNodeMap)[nd];

        CityNode* cnp = dynamic_cast<CityNode*>(n1.get());
        if (cnp)
            countries[cnp->country()].push_back(std::make_pair(nd, cnp));
    }

    // create work items
    EdgeList edges_to_delete;
    std::list<std::pair<Graph::Node, Graph::Node>> edges_to_add;

    std::unique_ptr<Config> config(new Config);
    double minBeta = config->get<double>("betaSkeleton.minBeta");
    assert(minBeta > 0.0);
    double maxBeta = config->get<double>("betaSkeleton.maxBeta");
    assert(maxBeta > 0.0);
    assert(maxBeta < 2.0);

    std::unique_ptr<InternetUsageStatistics> stat(new InternetUsageStatistics(PredefinedValues::dbFilePath()));

    // fill work queue
    for (auto country : countries) {
        std::string countryName = country.first;
        double percentInetUsers = (*stat)[countryName] / 100.0;
        double beta = percentInetUsers * minBeta + (1.0 - percentInetUsers) * maxBeta;

        for (Node& nd1 : country.second)
            for (Node& nd2 : country.second) {
                if (nd1.second == nd2.second || nd1.second->id() > nd2.second->id())
                    continue;

                if (beta >= 1.0) {
                    Graph::Edge edge = findEdge(*_graph, nd1.first, nd2.first);
                    if (edge != INVALID && !isBetaSkeletonEdgeGreaterEqualThanOne(edge, beta))
                        edges_to_delete.push_back(edge);
                } else if (isBetaSkeletonEdgeSmallerThanOne(nd1.first, nd2.first, beta))
                    edges_to_add.push_back(std::make_pair(nd1.first, nd2.first));
                else {
                    Graph::Edge edge = findEdge(*_graph, nd1.first, nd2.first);
                    if (edge != INVALID)
                        edges_to_delete.push_back(edge);
                }
            }
    }

    // add edges
    for (auto pair : edges_to_add)
        _graph->addEdge(pair.first, pair.second);

    // erase edges
    for (EdgeList::iterator edge = edges_to_delete.begin(); edge != edges_to_delete.end(); ++edge)
        _graph->erase(*edge);
}

void BetaSkeletonFilter::filterBetaSkeletonEdges() {
    // create global gabriel graph
    generalGabrielFilter();

    // per country filtering with beta skeleton
    perCountryBetaFilter();
}

Graph_Ptr BetaSkeletonFilter::getGraph(void) {
    return _graph;
}

NodeMap_Ptr BetaSkeletonFilter::getNodeMap(void) {
    return _nodeGeoNodeMap;
}

bool BetaSkeletonFilter::isBetaSkeletonEdgeGreaterEqualThanOne(Graph::Edge& edge, double beta) {
    // get all other points adjacent to node endpoints
    using namespace lemon;
    typedef ListGraph::Node Node;

    Node u = _graph->u(edge);
    Node v = _graph->v(edge);

    // determine circle center
    GeographicNode_Ptr n1 = (*_nodeGeoNodeMap)[u];
    GeographicNode_Ptr n2 = (*_nodeGeoNodeMap)[v];

    if (dynamic_cast<SeaCableNode*>(n1.get()) || dynamic_cast<SeaCableNode*>(n2.get()))
        return false;

    assert(beta >= 1.0);

    double theta = asin(1.0 / beta);

    typedef std::set<Node> NodeSet;
    NodeSet adjacentNodesU;
    NodeSet nodesToTest;

    for (ListGraph::IncEdgeIt it(*_graph, u); it != INVALID; ++it) {
        Node oppositeNode = _graph->oppositeNode(u, it);
        if (oppositeNode != v)
            adjacentNodesU.insert(oppositeNode);
    }

    // intersection of adjacent nodes of u and v has to be tested
    for (ListGraph::IncEdgeIt it(*_graph, v); it != INVALID; ++it) {
        Node oppositeNode = _graph->oppositeNode(v, it);
        if (oppositeNode != u && adjacentNodesU.find(oppositeNode) != adjacentNodesU.end())
            nodesToTest.insert(oppositeNode);
    }

    for (NodeSet::iterator n = nodesToTest.begin(); n != nodesToTest.end(); ++n) {
        GeographicNode_Ptr n3 = (*_nodeGeoNodeMap)[*n];
        bool isSeacable = dynamic_cast<SeaCableNode*>(n3.get()) != nullptr;

        if (!testTheta(n1, n3, n2, theta) && !isSeacable)
            return false;
    }

    return true;
}

bool BetaSkeletonFilter::isBetaSkeletonEdgeSmallerThanOne(Graph::Node& u, Graph::Node& v, double beta) {
    // get all other points adjacent to node endpoints
    using namespace lemon;
    typedef ListGraph::Node Node;

    // determine circle center
    GeographicNode_Ptr& n1 = (*_nodeGeoNodeMap)[u];
    GeographicNode_Ptr& n2 = (*_nodeGeoNodeMap)[v];

    if (dynamic_cast<SeaCableNode*>(n1.get()) || dynamic_cast<SeaCableNode*>(n2.get()))
        return false;

    double theta = M_PI - asin(beta);

    Bfs<Graph> bfs(*_graph);
    bfs.init();
    bfs.addSource(u);
    assert(bfs.processNextNode() == u);

    while (!bfs.emptyQueue()) {
        Node next = bfs.processNextNode();
        if (next == u || next == v)
            continue;
        GeographicNode_Ptr& n3 = (*_nodeGeoNodeMap)[next];

        bool isSeacable = dynamic_cast<SeaCableNode*>(n3.get()) != nullptr;

        if (!testTheta(n1, n3, n2, theta) && !isSeacable)
            return false;
    }

    return true;
}

// test angle prq
bool BetaSkeletonFilter::testTheta(GeographicNode_Ptr& p, GeographicNode_Ptr& r, GeographicNode_Ptr& q, double theta) {
    double a = sphericalDist(p, r);
    double b = sphericalDist(q, r);
    double c = sphericalDist(p, q);
    double C = Util::ihs((Util::hs(c) - Util::hs(a - b)) / (sin(a) * sin(b)));

    if (C >= theta)
        return false;
    else
        return true;
}
