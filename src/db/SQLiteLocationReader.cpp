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

#include "SQLiteLocationReader.hpp"

#include <boost/log/trivial.hpp>
#include <cassert>
#include <iostream>
#include <sstream>

SQLiteLocationReader::SQLiteLocationReader(std::string dbPath, int populationThreshold)
    : _populationThreshold(populationThreshold) {
    int retval = sqlite3_open(dbPath.c_str(), &_sqliteDB);

    // If connection failed, handle returns NULL
    if (retval) {
        BOOST_LOG_TRIVIAL(error) << "Database connection failed for SQLiteLocationReader";
    }
    BOOST_LOG_TRIVIAL(info) << "SQLite connection to " << dbPath << " successfully established in SQLiteLocationReader!";

    std::string queryString(
        " SELECT geo.Name AS Name,"
        "        geo.Latitude AS Latitude,"
        "        geo.Longitude AS Longitude,"
        "        geo.population AS Population,"
        "        ci.country AS Country,"
        "        ci.continent AS Continent"
        " FROM geoname as geo, countryinfo as ci"
        " WHERE geo.population >= ?"
        "   AND geo.country_code = ci.iso"
        " ORDER BY Name");

    sqlite3_prepare_v2(_sqliteDB, queryString.c_str(), queryString.length(), &_stmt, NULL);
    sqlite3_bind_int(_stmt, 1, _populationThreshold);
    retval = sqlite3_step(_stmt);
    if (retval == SQLITE_ROW)
        _rowAvailable = true;
    else
        _rowAvailable = false;
}

CityNode SQLiteLocationReader::getNext() {
    assert(_rowAvailable);

    std::stringstream ss;
    std::string name(reinterpret_cast<const char*>(sqlite3_column_text(_stmt, 0)));

    ss << sqlite3_column_text(_stmt, 1);
    double latitude;
    ss >> latitude;
    ss.str("");
    ss.clear();

    ss << sqlite3_column_text(_stmt, 2);
    double longitude;
    ss >> longitude;

    double population(sqlite3_column_double(_stmt, 3));
    std::string country(reinterpret_cast<const char*>(sqlite3_column_text(_stmt, 4)));
    std::string continent(reinterpret_cast<const char*>(sqlite3_column_text(_stmt, 5)));

    CityNode ci(0, name, latitude, longitude, population, country, continent);

    if (sqlite3_step(_stmt) == SQLITE_ROW)
        _rowAvailable = true;
    else {
        _rowAvailable = false;
    }
    return ci;
}
