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

#ifndef POPULATIONDENSITYLINECALCULATOR_HPP
#define POPULATIONDENSITYLINECALCULATOR_HPP

#include "db/PopulationDensityReader.hpp"
#include "geo/GeographicPosition.hpp"
#include "util/Util.hpp"
#include <memory>
#include <utility>
#include <cassert>

typedef std::vector<double> DensityVector;
typedef std::shared_ptr<DensityVector> DensityVector_Ptr;

class PopulationDensityLineCalculator;
typedef std::shared_ptr<PopulationDensityLineCalculator> PopulationDensityLineCalculator_Ptr;

class PopulationDensityLineCalculator {
   public:
    PopulationDensityLineCalculator(PopulationDensityReader_Ptr reader);

    ~PopulationDensityLineCalculator();

    DensityVector_Ptr getDensityLineBetween(GeographicPosition& p1, GeographicPosition& p2);

   protected:
   private:
    double calcDist(GeographicPosition& p1, GeographicPosition& p2);

    GeographicPosition getIntermediatePointAt(GeographicPosition& p1,
                                              GeographicPosition& p2,
                                              double distance,
                                              double percent);

    void appendLinePoint(DensityVector_Ptr result, GeographicPosition& p);

    PopulationDensityReader_Ptr _reader;
};

#endif  // POPULATIONDENSITYLINECALCULATOR_HPP
