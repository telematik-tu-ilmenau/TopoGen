/*
 * Copyright (c) 2013-2014, Michael Grey and Markus Theil
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

#ifndef POPULATIONDENSITYREADER_HPP
#define POPULATIONDENSITYREADER_HPP

#include <vector>
#include <memory>

class PopulationDensityReader;
typedef std::shared_ptr<PopulationDensityReader> PopulationDensityReader_Ptr;

// first:  row
// second: col
typedef std::pair<int, int> CellPosition;

struct FileHeader {
    int ncols;
    int nrows;
    int xllcorner;
    int yllcorner;
    double cellsize;
    long NODATA_value;
};

class PopulationDensityReader {
   public:
    PopulationDensityReader();
    double valueAt(double lat, double lon);
    double valueAt(int x, int y);
    double cellsize(void);
    ~PopulationDensityReader();
    CellPosition getRasterPosition(double lat, double lon);

   protected:
   private:
    void parseHeader();
    void readData();
    size_t getDataSize();
    CellPosition calcDataPosition(double lat, double lon);

    int _file;
    FileHeader _header;

    double* _data;
};

#endif  // POPULATIONDENSITYREADER_HPP