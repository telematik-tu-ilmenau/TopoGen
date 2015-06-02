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

#include "config/Config.hpp"
#include "config/PredefinedValues.hpp"
#include "db/InternetUsageStatistics.hpp"
#include "db/LandingPointReader.hpp"
#include "db/SQLiteLocationReader.hpp"
#include "db/SQLiteLocationReader.hpp"
#include "db/SubmarineCable.hpp"
#include "geo/CityNode.hpp"
#include "geo/GeographicNode.hpp"
#include "geo/GeometricHelpers.hpp"
#include "geo/SeaCableLandingPoint.hpp"
#include "geo/SeaCableNode.hpp"
#include "NodeImporter.hpp"
#include "geo/SeaCableEdge.hpp"
#include <fstream>
#include <string>
#include <random>
#include <cassert>
#include <boost/log/trivial.hpp>

NodeImporter::NodeImporter(void)
    : _nodenumber(0), _locations(new Locations), _dbFilename(PredefinedValues::dbFilePath()), _fallbackProjection() {
}

void NodeImporter::addNode(GeographicNode_Ptr node) {
    _locations->push_back(node);
}

Locations_Ptr NodeImporter::getLocations(void) {
    return _locations;
}

void NodeImporter::importCitiesFromFile(void) {
    /*
      ASSUME TOPOVIEW MAP EXPORT FILE FORMAT [STR , LAT , LON]
    */
    std::unique_ptr<Config> config(new Config);
    std::ifstream inputFile((config->get<std::string>("debug.inputNodePath")).c_str());
    assert(inputFile.good());

    std::string line;
    while (std::getline(inputFile, line)) {
        // extract latitude and longitude
        std::stringstream linestream(line);
        std::vector<std::string> elems;
        std::string value;
        while (std::getline(linestream, value, ',')) {
            elems.push_back(value);
        }
        double lon = atof(elems.back().c_str());
        elems.pop_back();
        double lat = atof(elems.back().c_str());

        // create node from scratch
        CityNode ci(_nodenumber, std::to_string(_nodenumber), lat, lon, 0.0, "United States");
        ++_nodenumber;
        GeographicNode_Ptr np(new CityNode(ci));
        addNode(np);
    }
}

void NodeImporter::importCities(const std::string& seedString) {
    /*
      READ CITY POSITIONS ON EARTH SURFACE
    */
    Config_Ptr config(new Config);

    Config_Ptr cityFilterConfig(config->subConfig("cityfilter"));
    int populationThreshold = cityFilterConfig->get<int>("citysizethreshold");

    auto lr = std::make_shared<SQLiteLocationReader>(_dbFilename, populationThreshold);

    typedef std::map<std::string, std::vector<CityNode>> CountryMap;
    std::unique_ptr<CountryMap> countries(new CountryMap);

    while (lr->hasNext()) {
        CityNode next = lr->getNext();
        (*countries)[next.country()].push_back(next);
    }

    std::unique_ptr<InternetUsageStatistics> stat(new InternetUsageStatistics(_dbFilename));

    typedef std::mt19937_64 RNG;
    std::seed_seq seed(seedString.begin(), seedString.end());
    std::unique_ptr<RNG> randGen(new RNG);
    randGen->seed(seed);
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    // filter cities
    for (auto country : *countries) {
        std::string countryName = country.first;
        auto& cityVec = country.second;
        double percentInetUsers = (*stat)[countryName] / 100.0;

        for (auto city : cityVec)
            if (dist(*randGen) <= percentInetUsers) {
                city.setId(_nodenumber);
                ++_nodenumber;
                GeographicNode_Ptr np(new CityNode(city));
                addNode(np);
            }
    }
}

GeographicNode_Ptr NodeImporter::findNearest(GeographicNode_Ptr& node) {
    GeographicNode_Ptr nearest = nullptr;
    double currentDist = std::numeric_limits<double>::max();
    for (auto otherNode : *_locations) {
        double dist = GeometricHelpers::sphericalDist(node, otherNode);
        if (dist < currentDist) {
            nearest = otherNode;
            currentDist = dist;
        }
    }
    return nearest;
}

void NodeImporter::importSeacableLandingPoints() {
    // add submarine cable landingpoints
    std::unique_ptr<LandingPointReader> lpr(new LandingPointReader(_dbFilename));

    while (lpr->hasNext()) {
        SeaCableLandingPoint next = lpr->getNext();
        next.setId(_nodenumber);
        ++_nodenumber;
        GeographicNode_Ptr lp(new SeaCableLandingPoint(next));

        // find nearest node
        GeographicNode_Ptr nnP = findNearest(lp);
        double dist = GeometricHelpers::sphericalDist(lp, nnP);
        if (dist > NodeImporter::DIST_TRESHOLD)
            addNode(lp);
        else {
            CityNode* cnp = dynamic_cast<CityNode*>(nnP.get());
            if (cnp)
                cnp->setSeaCableLandingPoint();
        }
    }
}

void NodeImporter::importSubmarineCableEdgesWaypoints() {
    std::unique_ptr<SubmarineCable> sc(new SubmarineCable(_dbFilename));

    while (sc->hasNext()) {
        SubmarineCableEdge edge = sc->getNext();
        if (edge.coord1 == edge.coord2)
            continue;

        GeographicNode_Ptr n1(new SeaCableNode(_nodenumber, edge.coord1.first, edge.coord1.second));
        ++_nodenumber;
        // find nearest node
        GeographicNode_Ptr nearestNode = findNearest(n1);

        SeaCableLandingPoint* slp = dynamic_cast<SeaCableLandingPoint*>(nearestNode.get());
        CityNode* cnp = dynamic_cast<CityNode*>(nearestNode.get());

        GeographicNode_Ptr nnP(new GeographicNode(*nearestNode));
        double dist = GeometricHelpers::sphericalDist(n1, nnP);
        if (!((slp || (cnp && cnp->isSeaCableLandingPoint())) && dist < NodeImporter::DIST_TRESHOLD))
            addNode(n1);
        else {
            auto rtn = _fallbackProjection.insert(std::make_pair(edge.coord1, nearestNode->coord()));
            assert(rtn.second == false || _fallbackProjection.at(edge.coord1) == nearestNode->coord());
        }

        GeographicNode_Ptr n2(new SeaCableNode(_nodenumber, edge.coord2.first, edge.coord2.second));
        ++_nodenumber;
        // find nearest node
        nearestNode = findNearest(n2);
        slp = dynamic_cast<SeaCableLandingPoint*>(nearestNode.get());
        cnp = dynamic_cast<CityNode*>(nearestNode.get());
        nnP = GeographicNode_Ptr(new GeographicNode(*nearestNode));
        dist = GeometricHelpers::sphericalDist(n2, nnP);
        if (!((slp || (cnp && cnp->isSeaCableLandingPoint())) && dist < NodeImporter::DIST_TRESHOLD))
            addNode(n2);
        else {
            auto rtn = _fallbackProjection.insert(std::make_pair(edge.coord2, nearestNode->coord()));
            assert(rtn.second == false || _fallbackProjection.at(edge.coord2) == nearestNode->coord());
        }
    }
}

void NodeImporter::importSubmarineCableEdges(BaseTopology_Ptr base_topo) {
    // add submarine cables waypoints
    std::unique_ptr<SubmarineCable> sc(new SubmarineCable(_dbFilename));

    unsigned int skipped = 0;
    while (sc->hasNext()) {
        SubmarineCableEdge edge = sc->getNext();
        if (edge.coord1 == edge.coord2) {
            ++skipped;
            continue;
        }

        GeographicPositionTuple c1 = std::make_pair(edge.coord1.first, edge.coord1.second);
        GeographicPositionTuple c2 = std::make_pair(edge.coord2.first, edge.coord2.second);

        if (_fallbackProjection.find(c1) != _fallbackProjection.end())
            c1 = _fallbackProjection.at(c1);
        if (_fallbackProjection.find(c2) != _fallbackProjection.end())
            c2 = _fallbackProjection.at(c2);
        if (c1 == c2) {
            ++skipped;
            continue;
        }

        GeographicNode_Ptr c1N(new GeographicNode(500, edge.coord1.first, edge.coord1.second));
        GeographicNode_Ptr c2N(new GeographicNode(500, edge.coord2.first, edge.coord2.second));

        GeographicNode_Ptr n1 = findNearest(c1N);
        GeographicNode_Ptr n2 = findNearest(c2N);

        Graph::Node u;
        Graph::Node v;

        u = base_topo->getGraph()->nodeFromId(n1->id());
        v = base_topo->getGraph()->nodeFromId(n2->id());

        GeographicEdge_Ptr edge_ptr(new SeaCableEdge);
        assert(u != v);
        base_topo->addEdge(u, v, edge_ptr);
    }

    BOOST_LOG_TRIVIAL(info) << "SubmarineCables: Skipped " << skipped
                            << " edge entries due to mapping to same Coordinate or invalid database info.";
}
