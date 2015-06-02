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

#include "PopulationDensityFilter.hpp"

#include <boost/log/trivial.hpp>
#include "config/Config.hpp"
#include "config/PredefinedValues.hpp"
#include "db/InternetUsageStatistics.hpp"
#include "db/SQLiteAreaPopulationReader.hpp"
#include "geo/CityNode.hpp"
#include "geo/GeometricHelpers.hpp"
#include "geo/GeographicPosition.hpp"
#include "geo/SeaCableLandingPoint.hpp"
#include "util/Util.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <lemon/list_graph.h>
#include <numeric>

PopulationDensityFilter::PopulationDensityFilter(BaseTopology_Ptr baseTopo)
    : _dbFilename(PredefinedValues::dbFilePath()), _baseTopo(baseTopo) {
}

void PopulationDensityFilter::filter(void) {
    using namespace lemon;
    std::unique_ptr<InternetUsageStatistics> inetStat(new InternetUsageStatistics(PredefinedValues::dbFilePath()));

    // crucial parameters
    std::unique_ptr<Config> config(new Config);
    const double MIN_LENGTH = config->get<double>("lengthFilter.minLength");
    const double POPULATION_THRESHOLD = config->get<double>("lengthFilter.populationThreshold");
    const double BETA = config->get<double>("lengthFilter.beta");

    // iterate over edges
    Graph& graph = *_baseTopo->getGraph();
    auto& nodeGeoNodeMap = *_baseTopo->getNodeMap();
    typedef std::list<Graph::Edge> Edgelist;
    Edgelist edges_to_delete;

    auto isValidNode = [](GeographicNode_Ptr& ptr) -> bool {
        CityNode* n1 = dynamic_cast<CityNode*>(ptr.get());
        SeaCableLandingPoint* n2 = dynamic_cast<SeaCableLandingPoint*>(ptr.get());
        return n1 != nullptr || n2 != nullptr;
    };

    for (ListGraph::EdgeIt it(graph); it != INVALID; ++it) {
        Graph::Node u = graph.u(it);
        Graph::Node v = graph.v(it);

        GeographicNode_Ptr nd1 = nodeGeoNodeMap[u];
        GeographicNode_Ptr nd2 = nodeGeoNodeMap[v];

        if (isValidNode(nd1) && isValidNode(nd2)) {
            GeographicPosition p1(nd1->lat(), nd1->lon());
            GeographicPosition p2(nd2->lat(), nd2->lon());

            // escape edges below specific length treshold
            double c = GeometricHelpers::sphericalDist(p1, p2);
            double c_km = GeometricHelpers::sphericalDistToKM(c);
            if (c_km < MIN_LENGTH) {
                continue;
            }

            // INIT Bounding box reader
            GeographicPositionTuple midPoint = GeometricHelpers::getMidPointCoordinates(p1, p2);
            GeographicPosition midPointPos(midPoint.first, midPoint.second);
            SQLiteAreaPopulationReader_Ptr areaReader(new SQLiteAreaPopulationReader(
                _dbFilename, midPoint.first, midPoint.second, GeometricHelpers::rad2deg(c)));

            double accPopulation = 0.0;
            while (areaReader->hasNext() && accPopulation <= POPULATION_THRESHOLD) {
                PopulatedPosition next = areaReader->getNext();

                // Nothing to accumulate, skip
                assert(next._population >= 0.0);
                if (next._population == 0.0) {
                    continue;
                }

                // test if the populated position is within a more sophisticated area (derived from a beta-skeleton
                // shape parameter)
                GeographicPosition toTest(next._lat, next._lon);
                double a = GeometricHelpers::sphericalDist(p1, toTest);
                double b = GeometricHelpers::sphericalDist(p2, toTest);
                double C = Util::ihs((Util::hs(c) - Util::hs(a - b)) / (sin(a) * sin(b)));

                // Point is out of area, skip
                const double theta = M_PI - asin(BETA);
                if (C < theta) {
                    continue;
                }

                // Weight by technology factor and additional weight
                double amountInetUsers = (*inetStat)[next._country] / 100.0;
                double popWeight = 1.0 - (GeometricHelpers::sphericalDist(toTest, midPointPos) /
                                          (0.5 * c));  // simply weight by distance to midpoint coordinate
                accPopulation += popWeight * next._population * pow(amountInetUsers, 2) * pow((MIN_LENGTH / c_km), 2);
            }

            if (accPopulation <= POPULATION_THRESHOLD) {
                edges_to_delete.push_back(it);
            }
        }
    }

    BOOST_LOG_TRIVIAL(info) << edges_to_delete.size() << " edges deleted by population density filter";

    for (Edgelist::iterator edge = edges_to_delete.begin(); edge != edges_to_delete.end(); ++edge)
        graph.erase(*edge);
}

void PopulationDensityFilter::filterByLength(void) {
    using namespace lemon;
    std::unique_ptr<InternetUsageStatistics> inetStat(new InternetUsageStatistics(PredefinedValues::dbFilePath()));

    // crucial parameters
    std::unique_ptr<Config> config(new Config);
    const double MIN_LENGTH = config->get<double>("lengthFilter.minLength");

    // iterate over edges
    Graph& graph = *_baseTopo->getGraph();
    auto& nodeGeoNodeMap = *_baseTopo->getNodeMap();
    typedef std::list<Graph::Edge> Edgelist;
    Edgelist edges_to_delete;

    auto isValidNode = [](GeographicNode_Ptr& ptr) -> bool {
        CityNode* n1 = dynamic_cast<CityNode*>(ptr.get());
        SeaCableLandingPoint* n2 = dynamic_cast<SeaCableLandingPoint*>(ptr.get());
        return n1 != nullptr || n2 != nullptr;
    };

    auto isCityNode = [](GeographicNode_Ptr& ptr) -> bool {
        CityNode* n1 = dynamic_cast<CityNode*>(ptr.get());
        return n1 != nullptr;
    };

    for (ListGraph::EdgeIt it(graph); it != INVALID; ++it) {
        Graph::Node u = graph.u(it);
        Graph::Node v = graph.v(it);

        GeographicNode_Ptr nd1 = nodeGeoNodeMap[u];
        GeographicNode_Ptr nd2 = nodeGeoNodeMap[v];

        if (isValidNode(nd1) && isValidNode(nd2)) {
            GeographicPosition p1(nd1->lat(), nd1->lon());
            GeographicPosition p2(nd1->lat(), nd1->lon());

            double amountInetUsers = 0.0;
            if (isCityNode(nd1) && isCityNode(nd2)) {
                std::string nd1c = static_cast<CityNode*>(nd1.get())->country();
                std::string nd2c = static_cast<CityNode*>(nd2.get())->country();
                amountInetUsers += (*inetStat)[nd1c] / 100.0;
                amountInetUsers += (*inetStat)[nd2c] / 100.0;
                amountInetUsers /= 2.0;
            } else if (isCityNode(nd1)) {
                std::string nd1c = static_cast<CityNode*>(nd1.get())->country();
                amountInetUsers += (*inetStat)[nd1c] / 100.0;
            } else if (isCityNode(nd2)) {
                std::string nd2c = static_cast<CityNode*>(nd2.get())->country();
                amountInetUsers += (*inetStat)[nd2c] / 100.0;
            } else
                continue;  // < skip, we won't filter edges between landing points

            // escape edges below specific length treshold
            assert(amountInetUsers < 1.0);
            double c = GeometricHelpers::sphericalDist(p1, p2);
            double c_km = GeometricHelpers::sphericalDistToKM(c);
            if (c_km > MIN_LENGTH * (1.0 + amountInetUsers)) {
                edges_to_delete.push_back(it);
            }
        }
    }
    // erase edges
    for (Edgelist::iterator edge = edges_to_delete.begin(); edge != edges_to_delete.end(); ++edge)
        graph.erase(*edge);
}
