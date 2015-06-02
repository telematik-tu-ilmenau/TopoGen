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

#ifndef KMLWRITER_HPP
#define KMLWRITER_HPP

#include "topo/base_topo/BaseTopology.hpp"
#include <lemon/list_graph.h>
#include <sstream>

class KMLWriter {
   public:
    KMLWriter(BaseTopology_Ptr baseTopo);
    virtual ~KMLWriter();

    void setEdgeColor(std::string hex, double alpha);

    void setPinColor(std::string hex, double alpha);

    void setSeacableColor(std::string hex, double alpha);

    void setSeacablePinColor(std::string hex, double alpha);

    void disableSeacablePins();
    void disableLocationsPins();

    void createKML(void);

    void write(const char* filename);

   private:
    BaseTopology_Ptr _baseTopo;
    Graph_Ptr _graph;
    NodeMap_Ptr _nodeInfos;
    std::string _pincolor;
    std::string _edgecolor;
    std::string _seacableColor;
    std::string _seacablePinColor;

    std::stringstream _kmlOut;

    bool _drawLocationPins;
    bool _drawSeacablePins;

    const std::string intToHex(int i);
    std::string alphaToHex(double alpha);
    std::string hexToKML(std::string hex);

    void drawCircleAt(double lat, double lon);

    KMLWriter(const KMLWriter&);
};

#endif
