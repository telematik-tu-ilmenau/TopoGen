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

#ifndef GEOGRAPHICNODE_HPP
#define GEOGRAPHICNODE_HPP

#include <vector>
#include <memory>

class GeographicNode {
   public:
    GeographicNode();
    GeographicNode(const GeographicNode& other);
    GeographicNode(int id, double lat, double lon);
    GeographicNode& operator=(const GeographicNode& other);
    virtual double lat();
    virtual double lon();
    virtual std::pair<double, double> coord();
    virtual int id();
    virtual void setId(int i);
    virtual void setLat(double lat);
    virtual void setLon(double lon);
    virtual void setInvalid();
    virtual bool isValid();

    // for usage as map key
    virtual bool operator<(const GeographicNode& other) const;
    virtual ~GeographicNode() {}

   protected:
    int _id;
    double _latitude;
    double _longitude;
    bool _valid;
};

typedef std::shared_ptr<GeographicNode> GeographicNode_Ptr;
typedef std::vector<GeographicNode_Ptr> Locations;
typedef std::shared_ptr<Locations> Locations_Ptr;

#endif  // class GeographicNode
