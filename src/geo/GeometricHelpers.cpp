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

#include "GeometricHelpers.hpp"

#include <algorithm>
#include <cmath>

double GeometricHelpers::deg2rad(double deg) {
    return deg * GeometricHelpers::DEG_TO_RAD;
}

// http://blog.julien.cayzac.name/2008/10/arc-and-distance-between-two-points-on.html
// use the law of haversines for numerical stability
double GeometricHelpers::sphericalDist(GeographicPosition& from, GeographicPosition& to) {
    double latitudeArc = (from.lat() - to.lat()) * GeometricHelpers::DEG_TO_RAD;
    double longitudeArc = (from.lon() - to.lon()) * GeometricHelpers::DEG_TO_RAD;
    double latitudeH = sin(latitudeArc * 0.5);
    latitudeH *= latitudeH;
    double lontitudeH = sin(longitudeArc * 0.5);
    lontitudeH *= lontitudeH;
    double tmp = cos(from.lat() * GeometricHelpers::DEG_TO_RAD) * cos(to.lat() * GeometricHelpers::DEG_TO_RAD);
    return 2.0 * asin(sqrt(latitudeH + tmp * lontitudeH));
}

double GeometricHelpers::sphericalDist(GeographicNode_Ptr& from, GeographicNode_Ptr& to) {
    GeographicPosition p1(from->lat(), from->lon());
    GeographicPosition p2(to->lat(), to->lon());
    return GeometricHelpers::sphericalDist(p1, p2);
}

GeographicPositionTuple GeometricHelpers::getMidPointCoordinates(GeographicNode_Ptr& from, GeographicNode_Ptr& to) {
    GeographicPosition p1(from->lat(), from->lon());
    GeographicPosition p2(to->lat(), to->lon());
    return GeometricHelpers::getMidPointCoordinates(p1, p2);
}

GeographicPositionTuple GeometricHelpers::getMidPointCoordinates(GeographicPosition& n1, GeographicPosition& n2) {
    // calculate midpoint
    // http://www.movable-type.co.uk/scripts/latlong.html

    double lat1 = deg2rad(n1.lat());
    double lon1 = deg2rad(n1.lon());
    double lat2 = deg2rad(n2.lat());

    double dLon = deg2rad(n2.lon() - n1.lon());

    double Bx = cos(lat2) * cos(dLon);
    double By = cos(lat2) * sin(dLon);
    double lat3 = atan2(sin(lat1) + sin(lat2), sqrt((cos(lat1) + Bx) * (cos(lat1) + Bx) + By * By));
    double lon3 = fmod(lon1 + atan2(By, cos(lat1) + Bx) + 3.0 * M_PI, 2.0 * M_PI) - M_PI;

    return std::make_pair(rad2deg(lat3), rad2deg(lon3));
}

double GeometricHelpers::rad2deg(double rad) {
    return rad * GeometricHelpers::RAD_TO_DEG;
}

double GeometricHelpers::sphericalDistToKM(double dist) {
    return dist * GeometricHelpers::EARTH_RADIUS_KM;
}
