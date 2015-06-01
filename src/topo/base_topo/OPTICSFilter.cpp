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

#include "OPTICSFilter.hpp"

#include "config/Config.hpp"
#include "geo/GeometricHelpers.hpp"
#include <algorithm>
#include <cassert>
#include <random>

OPTICSFilter::OPTICSFilter(Locations_Ptr& locations, double eps, unsigned int minPts, double epsDBSCAN)
    : _locations(locations), _eps(eps), _minPts(minPts), _epsDBSCAN(epsDBSCAN) {
    for (GeographicNode_Ptr& node : *_locations) {
        auto opticsObj = OPTICSObject_Ptr(new OPTICSObject(node));
        _opticsObjects.insert(std::make_pair(node->id(), opticsObj));
        _unprocessedObjects.push_back(opticsObj);
    }
}

OPTICSFilter::OPTICSObjectVector_Ptr OPTICSFilter::getEpsilonNeighbors(OPTICSObject_Ptr primaryObject) {
    OPTICSObjectVector_Ptr epsilonNeighbors(new OPTICSObjectVector);

    for (auto opticsObjectPair : _opticsObjects) {
        OPTICSObject_Ptr obj = opticsObjectPair.second;
        if (obj->node->id() == primaryObject->node->id())
            continue;

        obj->tmpDistance = GeometricHelpers::sphericalDist(primaryObject->node, obj->node);

        if (obj->tmpDistance <= _eps)
            epsilonNeighbors->push_back(obj);
    }

    if (epsilonNeighbors->size() >= _minPts) {
        std::sort(epsilonNeighbors->begin(), epsilonNeighbors->end(), OPTICSFilter::compareOPTICSObjectsTmp);
        primaryObject->coreDistance = (*epsilonNeighbors)[_minPts - 1]->tmpDistance;
    }

    return epsilonNeighbors;
}

void OPTICSFilter::updateSeeds(OPTICSObjectVector_Ptr neighbors,
                               OPTICSObject_Ptr centerObject,
                               OPTICSObjectVector_Ptr seeds) {
    double coreDistance = centerObject->coreDistance;
    for (OPTICSObject_Ptr& otherObject : *neighbors) {
        if (otherObject->processed == false) {
            assert(centerObject->node);
            assert(otherObject->node);

            double directDistance = GeometricHelpers::sphericalDist(centerObject->node, otherObject->node);
            double newReachabilityDistance = std::max(coreDistance, directDistance);

            if (otherObject->reachabilityDistance == UNDEFINED_DISTANCE) {
                otherObject->reachabilityDistance = newReachabilityDistance;
                seeds->push_back(otherObject);
            } else if (newReachabilityDistance < otherObject->reachabilityDistance)
                otherObject->reachabilityDistance = newReachabilityDistance;
        }
    }
    std::sort(seeds->begin(), seeds->end(), OPTICSFilter::compareOPTICSObjects);
}

void OPTICSFilter::expandClusterOrder(OPTICSObject_Ptr node) {
    OPTICSObjectVector_Ptr epsilonNeighbors = getEpsilonNeighbors(node);
    node->processed = true;

    // output p to the ordered list
    _orderedObjects.push_back(node);

    OPTICSObjectVector_Ptr seeds(new OPTICSObjectVector);
    if (node->coreDistance != UNDEFINED_DISTANCE) {
        updateSeeds(epsilonNeighbors, node, seeds);

        while (!seeds->empty()) {
            OPTICSObject_Ptr currentObject = seeds->back();
            seeds->pop_back();

            OPTICSObjectVector_Ptr currentObjectEpsilonNeighbors = getEpsilonNeighbors(currentObject);

            // output q to the ordered list
            if (currentObject->processed == false) {
                _orderedObjects.push_back(currentObject);
                currentObject->processed = true;
            } else
                continue;

            if (currentObject->coreDistance != UNDEFINED_DISTANCE) {
                updateSeeds(currentObjectEpsilonNeighbors, currentObject, seeds);
            }
        }
    }
}

void OPTICSFilter::extractDBSCANClustering(const std::string& seedString) {
    std::vector<OPTICSObject_Ptr> currentCluster;

    // needed for use of uniform dist
    typedef std::mt19937_64 RNG;
    std::seed_seq seed(seedString.begin(), seedString.end());
    std::unique_ptr<RNG> randGen(new RNG);
    randGen->seed(seed);

    // we start with cluster 1 and use 0 as noise
    int clusterID = 1;
    for (OPTICSObject_Ptr& obj : _orderedObjects) {
        if (obj->reachabilityDistance > _epsDBSCAN) {
            if (obj->coreDistance <= _epsDBSCAN) {
                // get random obj from currentCluster and append to locations
                if (currentCluster.empty() == false) {
                    std::uniform_int_distribution<int> elemDist(0, currentCluster.size() - 1);
                    OPTICSObject_Ptr randObj = currentCluster.at(elemDist(*randGen));
                    _locations->push_back(randObj->node);
                }
                currentCluster.clear();
                clusterID++;
                obj->clusterId = clusterID;
                currentCluster.push_back(obj);
            } else {
                obj->clusterId = 0;  // noise
                _locations->push_back(obj->node);
            }
        } else {
            obj->clusterId = clusterID;
            currentCluster.push_back(obj);
        }
    }
}

void OPTICSFilter::filter(const std::string& seedString) {
    auto it = _unprocessedObjects.begin();
    while (_unprocessedObjects.empty() == false) {
        OPTICSObject_Ptr node = *it;

        if (node->processed) {
            it = _unprocessedObjects.erase(it);
            continue;
        }

        expandClusterOrder(node);
    }

    assert(_orderedObjects.size() == _opticsObjects.size());

    // extract clusters via DBSCAN
    _locations->clear();

    extractDBSCANClustering(seedString);
}
