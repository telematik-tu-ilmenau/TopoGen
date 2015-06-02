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
#include "geo/GeographicPosition.hpp"
#include "geo/GeometricHelpers.hpp"
#include "topo/base_topo/CGALPrimitives.hpp"
#include <vector>
#include <cmath>
#include <ostream>

typedef std::vector<GeographicPositionTuple> GeographicPositionVector;

static void kmlDrawCircle(double latitude, double longitude, int segments, std::ostream& kmlOut);

void KMLWriter::drawCircleAt(double latitude, double longitude) {
    _kmlOut << "<Placemark>\n";

    _kmlOut << "<styleUrl>"
            << "#styleDefault"
            << "</styleUrl>\n";
    int segments = 50;
    kmlDrawCircle(latitude, longitude, segments, _kmlOut);

    _kmlOut << "</Placemark>\n";
}

// http://williams.best.vwh.net/avform.htm#LL: Lat/lon given radial and distance,
// Matlab names this reckon
static GeographicPositionTuple reckon(double latitude, double longitude, double range, double azimuth) {
    latitude = latitude * GeometricHelpers::DEG_TO_RAD;
    longitude = longitude * GeometricHelpers::DEG_TO_RAD;
    range = range * GeometricHelpers::DEG_TO_RAD;
    azimuth = azimuth;

    double lat = asin(sin(latitude) * cos(range) + cos(latitude) * sin(range) * cos(azimuth));
    double dlon = atan2(sin(azimuth) * sin(range) * cos(latitude), cos(range) - sin(latitude) * sin(lat));
    double lon = fmod(longitude - dlon + M_PI, 2.0 * M_PI) - M_PI;

    return std::make_pair(lat * GeometricHelpers::RAD_TO_DEG, lon * GeometricHelpers::RAD_TO_DEG);
}

static void kmlDrawCircle(double latitude, double longitude, int segments, std::ostream& kmlOut) {
    kmlOut << "<Polygon>\n"
           << "<outerBoundaryIs>\n"
           << "<LinearRing>\n"
           << "<coordinates>\n";

    double rangeInDegrees = 0.1875;

    for (int i = 0; i < segments; ++i) {
        auto pos = reckon(latitude, longitude, rangeInDegrees, 2.0 * M_PI / segments * i);
        kmlOut << pos.second << "," << pos.first << "\n";
    }

    auto pos = reckon(latitude, longitude, rangeInDegrees, 0.0);
    kmlOut << pos.second << "," << pos.first << "\n";

    kmlOut << "</coordinates>\n"
           << "</LinearRing>\n"
           << "</outerBoundaryIs>\n"
           << "</Polygon>\n";
}
