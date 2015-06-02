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

#include "KMLWriter.hpp"
#include "geo/GeographicNode.hpp"
#include "geo/SeaCableLandingPoint.hpp"
#include "geo/CityNode.hpp"
#include "geo/SeaCableNode.hpp"
#include "geo/SeaCableEdge.hpp"
#include <string>
#include <cassert>
#include <regex.h>
#include <regex>
#include <fstream>
#include <algorithm>
#include <iterator>

KMLWriter::KMLWriter(BaseTopology_Ptr baseTopo)
    : _baseTopo(baseTopo), _graph(_baseTopo->getGraph()), _nodeInfos(_baseTopo->getNodeMap()), _pincolor(), _edgecolor(), _seacableColor(), _seacablePinColor(), _kmlOut() {
    setEdgeColor("ffffff", 1.0);
    setPinColor("ffffff", 1.0);
}

KMLWriter::~KMLWriter() {
}

void KMLWriter::write(const char* filename) {
    std::ofstream kmlFile(filename);
    std::copy(std::istreambuf_iterator<char>(_kmlOut), std::istreambuf_iterator<char>(),
              std::ostreambuf_iterator<char>(kmlFile));
    kmlFile.close();
}

void KMLWriter::createKML() {
    _kmlOut << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";

    _kmlOut << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n";

    _kmlOut << "<Document>\n";

    {
        // style section
        _kmlOut << "<Style id=\"styleDefault\">\n";

        {
            // set iconstyle
            _kmlOut << "<IconStyle>\n";
            _kmlOut << "<color>" << _pincolor << "</color>";
            _kmlOut << "</IconStyle>\n";
        }

        {
            // set iconstyle
            _kmlOut << "<LineStyle>\n";
            _kmlOut << "<color>" << _edgecolor << "</color>\n";
            _kmlOut << "<colorMode>normal</colorMode>\n";
            _kmlOut << "<width>2</width>\n";
            _kmlOut << "</LineStyle>\n";
        }

        _kmlOut << "</Style>\n";
    }

    {
        // style section
        _kmlOut << "<Style id=\"seacableStyle\">\n";

        {
            // set iconstyle
            _kmlOut << "<IconStyle>\n";
            _kmlOut << "<color>" << _seacablePinColor << "</color>";
            _kmlOut << "</IconStyle>\n";
        }

        {
            // set iconstyle
            _kmlOut << "<LineStyle>\n";
            _kmlOut << "<color>" << _seacableColor << "</color>\n";
            _kmlOut << "<colorMode>normal</colorMode>\n";
            _kmlOut << "<width>2</width>\n";
            _kmlOut << "</LineStyle>\n";
        }

        _kmlOut << "</Style>\n";
    }

    // iterate over all edges
    using namespace lemon;
    for (ListGraph::EdgeIt edge(*_graph); edge != lemon::INVALID; ++edge) {
        GeographicNode_Ptr n1 = (*_nodeInfos)[_graph->u(edge)];
        GeographicNode_Ptr n2 = (*_nodeInfos)[_graph->v(edge)];
        if (!n1->isValid() || !n2->isValid())
            continue;

        _kmlOut << "<Placemark>\n";

        bool seacable = false;

        EdgeMap_Ptr edgeMap = _baseTopo->getEdgeMap();
        GeographicEdge_Ptr edge_Ptr = (*edgeMap)[edge];

        if (dynamic_cast<SeaCableEdge*>(edge_Ptr.get()))
            seacable = true;

        _kmlOut << "<styleUrl>";
        if (seacable)
            _kmlOut << "#seacableStyle";
        else
            _kmlOut << "#styleDefault";
        _kmlOut << "</styleUrl>\n";

        _kmlOut << "<LineString>\n";

        _kmlOut << "<tessellate>1</tessellate>\n";
        _kmlOut << "<extrude>1</extrude>\n";

        // insert actual coordinates
        _kmlOut << "<coordinates>";
        _kmlOut << n1->lon() << "," << n1->lat() << ",0 ";
        _kmlOut << n2->lon() << "," << n2->lat() << ",0";
        _kmlOut << "</coordinates>\n";
        _kmlOut << "</LineString>\n";
        _kmlOut << "</Placemark>\n";
    }

    // iterate over all nodes
    using namespace lemon;
    for (ListGraph::NodeIt node(*_graph); node != lemon::INVALID; ++node) {
        GeographicNode_Ptr place = (*_nodeInfos)[node];
        if (!place->isValid())
            continue;

        _kmlOut << "<Placemark>\n";

        if (dynamic_cast<CityNode*>(place.get()) != nullptr ||
            dynamic_cast<SeaCableLandingPoint*>(place.get()) != nullptr) {
            _kmlOut << "<name>";

            if (dynamic_cast<CityNode*>(place.get()) != nullptr) {
                auto* cityPlace = dynamic_cast<CityNode*>(place.get());
                _kmlOut << cityPlace->name();
            }

            if (dynamic_cast<SeaCableLandingPoint*>(place.get()) != nullptr) {
                auto* cityPlace = dynamic_cast<SeaCableLandingPoint*>(place.get());
                _kmlOut << cityPlace->name();
            }

            _kmlOut << "</name>\n";
        }

        _kmlOut << "<styleUrl>";

        if (dynamic_cast<SeaCableNode*>(place.get()))
            _kmlOut << "#seacableStyle";
        else
            _kmlOut << "#styleDefault";

        _kmlOut << "</styleUrl>\n";

        _kmlOut << "<Point>\n";

        // insert actual coordinates
        _kmlOut << "<coordinates>";
        _kmlOut << place->lon() << "," << place->lat() << ",0";
        _kmlOut << "</coordinates>\n";

        _kmlOut << "</Point>\n";

        _kmlOut << "</Placemark>\n";
    }

    _kmlOut << "</Document>\n";
    _kmlOut << "</kml>\n";
}

const std::string KMLWriter::intToHex(int i) {
    std::ostringstream oss;
    oss << std::hex << i;
    return oss.str();
}

void KMLWriter::setEdgeColor(std::string hex, double alpha) {
    _edgecolor = alphaToHex(alpha).append(hexToKML(hex));
}

void KMLWriter::setPinColor(std::string hex, double alpha) {
    _pincolor = alphaToHex(alpha).append(hexToKML(hex));
}

void KMLWriter::setSeacableColor(std::string hex, double alpha) {
    _seacableColor = alphaToHex(alpha).append(hexToKML(hex));
}

void KMLWriter::setSeacablePinColor(std::string hex, double alpha) {
    _seacablePinColor = alphaToHex(alpha).append(hexToKML(hex));
}

std::string KMLWriter::alphaToHex(double alpha) {
    int a_i = static_cast<int>(alpha * 255.0);
    return intToHex(a_i);
}

std::string KMLWriter::hexToKML(std::string hex) {
    assert(hex.length() == 6);

    std::regex colorCheck("^[0-9a-fA-F]{6}$", std::regex_constants::extended);
    assert(std::regex_match(hex, colorCheck) == true);

    std::string r;
    std::string g;
    std::string b;

    r = hex.substr(0, 2);
    g = hex.substr(2, 2);
    b = hex.substr(4, 2);

    std::string kmlHex;

    kmlHex.append(b);
    kmlHex.append(g);
    kmlHex.append(r);

    return kmlHex;
}
