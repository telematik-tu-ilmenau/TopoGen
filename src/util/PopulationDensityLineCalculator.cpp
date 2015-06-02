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

#include "PopulationDensityLineCalculator.hpp"
#include "geo/GeometricHelpers.hpp"
#include <cmath>
#include <iostream>

PopulationDensityLineCalculator::PopulationDensityLineCalculator(PopulationDensityReader_Ptr reader) : _reader(reader) {
}

PopulationDensityLineCalculator::~PopulationDensityLineCalculator() {
}

GeographicPosition PopulationDensityLineCalculator::getIntermediatePointAt(GeographicPosition& p1, GeographicPosition& p2, double distance, double percent) {
    using namespace GeometricHelpers;


    double A = sin((1 - percent) * distance) / sin(distance);
    double B = sin(percent * distance) / sin(distance);
    double lat1 = deg2rad(p1.lat());
    double lon1 = deg2rad(p1.lon());

    double lat2 = deg2rad(p2.lat());
    double lon2 = deg2rad(p2.lon());
    double x = A * cos(lat1) * cos(lon1) + B * cos(lat2) * cos(lon2);
    double y = A * cos(lat1) * sin(lon1) + B * cos(lat2) * sin(lon2);
    double z = A * sin(lat1) + B * sin(lat2);

    GeographicPosition p(rad2deg(atan2(z, sqrt(x * x + y * y))), rad2deg(atan2(y, x)));

    return p;
}

DensityVector_Ptr PopulationDensityLineCalculator::getDensityLineBetween(GeographicPosition& p1, GeographicPosition& p2) {
    using namespace GeometricHelpers;
    assert(Util::checkBounds(p1));
    assert(Util::checkBounds(p2));

    DensityVector_Ptr result(new DensityVector);

    // Aviation Formulas
    // Intermediate Points on a great circle
    // http://williams.best.vwh.net/avform.htm#Crs
    double distance = sphericalDist(p1, p2);
    double increment = deg2rad(_reader->cellsize()) / distance;

    for (double percent = 0.0; percent <= 1.0; percent += increment) {
        GeographicPosition p = getIntermediatePointAt(p1, p2, distance, percent);
        appendLinePoint(result, p);
    }

    return result;
}

void PopulationDensityLineCalculator::appendLinePoint(DensityVector_Ptr result, GeographicPosition& p) {
    // get raster points on array
    double density_val = _reader->valueAt(p.lat(), p.lon());
    if (density_val < 0.0)
        density_val = 0.0;
    result->push_back(density_val);
}
