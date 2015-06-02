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

#include "LandingPointReader.hpp"
#include <cassert>
#include <iostream>
#include <boost/log/trivial.hpp>

LandingPointReader::LandingPointReader(std::string dbName) {
    int retval = sqlite3_open(dbName.c_str(), &_sqliteDB);

    // If connection failed, handle returns NULL
    if (retval) {
        BOOST_LOG_TRIVIAL(error) << "Database connection failed in LandingPointReader!";
    }
    BOOST_LOG_TRIVIAL(info) << "SQLite connection to " << dbName << " successfully established in LandingPointReader!";

    std::string queryString("SELECT id, latitude, longitude, name, country from landingpoints");

    sqlite3_prepare_v2(_sqliteDB, queryString.c_str(), queryString.length(), &_stmt, NULL);
    retval = sqlite3_step(_stmt);
    _rowAvailable = false;
    if (retval == SQLITE_ROW)
        _rowAvailable = true;
    else
        _rowAvailable = false;
}

SeaCableLandingPoint LandingPointReader::getNext() {
    int id = sqlite3_column_int(_stmt, 0);
    double lat = sqlite3_column_double(_stmt, 1);
    double lon = sqlite3_column_double(_stmt, 2);
    std::string name = reinterpret_cast<const char*>(sqlite3_column_text(_stmt, 3));
    // std::string country = reinterpret_cast<const char*>(sqlite3_column_text(_stmt, 4));

    SeaCableLandingPoint landpoint(id, lat, lon, name);

    int retval = sqlite3_step(_stmt);
    if (retval == SQLITE_ROW)
        _rowAvailable = true;
    else
        _rowAvailable = false;

    return landpoint;
}

std::string LandingPointReader::getContinentLandingPoint(std::string name) {
    sqlite3_stmt* stmt;

    std::string queryString =
        "SELECT continent from countryinfo as ci, rel_landingpoint_to_countryinfo as lp where lp.country_landing = ? "
        "AND lp.country_countryinfo = ci.country";

    sqlite3_prepare_v2(_sqliteDB, queryString.c_str(), queryString.length(), &stmt, NULL);

    assert(sqlite3_bind_text(stmt, 1, name.c_str(), name.size(), NULL) == SQLITE_OK);
    int retval = sqlite3_step(stmt);
    std::string result;
    if (retval == SQLITE_ROW)
        result = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));

    sqlite3_finalize(stmt);

    return result;
}

LandingPointReader::~LandingPointReader() {
    sqlite3_finalize(_stmt);
    sqlite3_close(_sqliteDB);
}
