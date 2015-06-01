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

#ifndef OPTICSFILTER_HPP
#define OPTICSFILTER_HPP

#include "geo/GeographicNode.hpp"
#include <limits>
#include <list>
#include <map>
#include <memory>

class OPTICSFilter;
typedef std::shared_ptr<OPTICSFilter> OPTICSFilter_Ptr;

class OPTICSFilter {
   public:
    // locations are reduced in size with this filter
    OPTICSFilter(Locations_Ptr& locations, double eps, unsigned int minPts, double epsDBSCAN);

    void filter(const std::string& seedString);

   protected:
   private:
    struct OPTICSObject {
        unsigned int id;
        GeographicNode_Ptr node;

        double reachabilityDistance;
        double coreDistance;
        double tmpDistance;
        bool processed;
        unsigned int clusterId;  /// < 0 for NOISE

        OPTICSObject(GeographicNode_Ptr& node)
            : id(node->id()),
              node(node),
              reachabilityDistance(UNDEFINED_DISTANCE),
              coreDistance(UNDEFINED_DISTANCE),
              tmpDistance(UNDEFINED_DISTANCE),
              processed(false),
              clusterId(0) {}
    };

    typedef std::shared_ptr<OPTICSObject> OPTICSObject_Ptr;
    typedef std::vector<OPTICSObject_Ptr> OPTICSObjectVector;
    typedef std::shared_ptr<OPTICSObjectVector> OPTICSObjectVector_Ptr;
    static constexpr double UNDEFINED_DISTANCE = std::numeric_limits<double>::infinity();

    OPTICSObjectVector_Ptr getEpsilonNeighbors(OPTICSObject_Ptr node);
    void updateSeeds(OPTICSObjectVector_Ptr neighbors, OPTICSObject_Ptr centerObject, OPTICSObjectVector_Ptr seeds);
    void expandClusterOrder(OPTICSObject_Ptr node);
    void extractDBSCANClustering(const std::string& seedString);

    static bool compareOPTICSObjects(const OPTICSObject_Ptr& a, const OPTICSObject_Ptr& b) {
        return a->reachabilityDistance > b->reachabilityDistance;
    }

    static bool compareOPTICSObjectsTmp(const OPTICSObject_Ptr& a, const OPTICSObject_Ptr& b) {
        return a->tmpDistance < b->tmpDistance;
    }

    Locations_Ptr _locations;
    double _eps;
    unsigned int _minPts;
    double _epsDBSCAN;

    std::map<unsigned int, OPTICSObject_Ptr> _opticsObjects;
    OPTICSObjectVector _unprocessedObjects;

    std::vector<OPTICSObject_Ptr> _orderedObjects;
};

#endif
