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

#include "CGALPrimitives.hpp"
#include "geo/GeometricHelpers.hpp"

Point_3 CGALPrimitives::getMidPoint(GeographicNode_Ptr n1, GeographicNode_Ptr n2) {
    using namespace GeometricHelpers;

    std::pair<double, double> midCoords = getMidPointCoordinates(n1, n2);

    double lat3 = midCoords.first;
    double lon3 = midCoords.second;

    return createPoint(rad2deg(lat3), rad2deg(lon3));
}

Point_3 CGALPrimitives::createPoint(double lat, double lon) {
    using namespace GeometricHelpers;

    const double EARTH_RADIUS = 1.0;

    double lat1 = deg2rad(lat);
    double lon1 = deg2rad(lon);
    double x1 = EARTH_RADIUS * sin(lat1) * cos(lon1);
    double y1 = EARTH_RADIUS * sin(lat1) * sin(lon1);
    double z1 = EARTH_RADIUS * cos(lat1);
    return Point_3(x1, y1, z1);
}
