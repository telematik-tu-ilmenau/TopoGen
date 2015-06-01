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

#ifndef BETASKELETONFILTER_HPP
#define BETASKELETONFILTER_HPP

#include "BaseTopology.hpp"
#include "CGALPrimitives.hpp"
#include "geo/CityNode.hpp"
#include "geo/GeographicNode.hpp"
#include "topo/Graph.hpp"
#include <lemon/list_graph.h>
#include <map>
#include <utility>

class BetaSkeletonFilter {
   public:
    BetaSkeletonFilter(BaseTopology_Ptr baseTopo);

    virtual ~BetaSkeletonFilter();

    void filterBetaSkeletonEdges();

    Graph_Ptr getGraph(void);

    NodeMap_Ptr getNodeMap(void);

    void generalGabrielFilter();

    void perCountryBetaFilter();

   private:
    bool isBetaSkeletonEdgeGreaterEqualThanOne(Graph::Edge& edge, double beta);
    bool isBetaSkeletonEdgeSmallerThanOne(Graph::Node& u, Graph::Node& v, double beta);
    bool testTheta(GeographicNode_Ptr& p, GeographicNode_Ptr& r, GeographicNode_Ptr& q, double theta);

    BaseTopology_Ptr _baseTopo;
    Graph_Ptr _graph;
    NodeMap_Ptr _nodeGeoNodeMap;
};

#endif  // BETASKELETONFILTER_HPP
