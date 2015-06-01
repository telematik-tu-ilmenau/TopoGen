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

#include "PopulationDensityReader.hpp"
#include "config/PredefinedValues.hpp"
#include <cassert>
#include <string>
#include <fstream>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <cstdint>
#include <cstring>
#include <cmath>

PopulationDensityReader::PopulationDensityReader(void) : _data(nullptr) {
    _file = open(PredefinedValues::popDensityFilePath().c_str(), O_RDONLY);
    assert(_file != -1);
    parseHeader();
    readData();
}

double PopulationDensityReader::cellsize(void) {
    return _header.cellsize;
}

CellPosition PopulationDensityReader::getRasterPosition(double lat, double lon) {
    CellPosition result;

    result.first = static_cast<int>((lat - _header.yllcorner) / _header.cellsize);
    result.second = static_cast<int>((lon - _header.xllcorner) / _header.cellsize);

    return result;
}

double PopulationDensityReader::valueAt(double lat, double lon) {
    CellPosition pos = calcDataPosition(lat, lon);
    int row = pos.first;
    int col = pos.second;

    if (row == -1)  // out of bounds
        return _header.NODATA_value;

    int offset = sizeof(FileHeader) / sizeof(double);
    return _data[offset + row * _header.ncols + col];
}

double PopulationDensityReader::valueAt(int row, int col) {
    if (row < 0 || _header.nrows <= row)  // out of bounds
        return _header.NODATA_value;
    if (col < 0 || _header.ncols <= col)  // out of bounds
        return _header.NODATA_value;

    int offset = sizeof(FileHeader) / sizeof(double);
    return _data[offset + row * _header.ncols + col];
}

PopulationDensityReader::~PopulationDensityReader(void) {
    munmap(_data, getDataSize());
}

void PopulationDensityReader::parseHeader(void) {
    ssize_t readSize = read(_file, &_header, sizeof(FileHeader));
    assert(readSize == sizeof(FileHeader));
    assert(_header.NODATA_value == -9999);
}

void PopulationDensityReader::readData() {
    size_t size = lseek(_file, 0, SEEK_END);
    assert(size == _header.ncols * _header.nrows * sizeof(double) + sizeof(FileHeader));
    _data = reinterpret_cast<double*>(mmap(NULL, getDataSize() + sizeof(FileHeader), PROT_READ, MAP_SHARED, _file, 0));
    assert(_data != MAP_FAILED);
}

size_t PopulationDensityReader::getDataSize() {
    return _header.nrows * _header.ncols * sizeof(double);
}

template <typename T>
class RangeCheck {
   public:
    RangeCheck(T lower, T upper) : _lowerBound(lower), _upperBound(upper) {}

    bool operator()(T arg) { return arg >= _lowerBound && arg <= _upperBound; }

   private:
    T _lowerBound;
    T _upperBound;
};

CellPosition PopulationDensityReader::calcDataPosition(double lat, double lon) {
    assert(lat >= -90.0);
    assert(lat <= 90.0);
    assert(lon >= -180.0);
    assert(lon <= 180.0);

    double xllcorner = _header.xllcorner;
    double yllcorner = _header.yllcorner;
    double xurcorner = _header.xllcorner + _header.cellsize * _header.ncols;
    double yurcorner = _header.yllcorner + _header.cellsize * _header.nrows;

    RangeCheck<double> inX(xllcorner, xurcorner);
    RangeCheck<double> inY(yllcorner, yurcorner);

    CellPosition result;

    // inside?
    if (inY(lat) && inX(lon)) {
        result.first = _header.nrows - 1 - static_cast<int>((lat - _header.yllcorner) / _header.cellsize);
        assert(result.first >= 0 && result.first < _header.nrows);
        result.second = static_cast<int>((lon - _header.xllcorner) / _header.cellsize);
        assert(result.second >= 0 && result.second < _header.ncols);
    }  // outside
    else {
        result.first = -1;
        result.second = -1;
    }

    return result;
}
