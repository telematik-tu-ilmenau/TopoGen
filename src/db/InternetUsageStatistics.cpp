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

#include "InternetUsageStatistics.hpp"

#include <boost/log/trivial.hpp>
#include <cassert>
#include <iostream>
#include <sstream>

InternetUsageStatistics::InternetUsageStatistics(std::string dbPath) {
    int retval = sqlite3_open(dbPath.c_str(), &_sqliteDB);

    // If connection failed, handle returns NULL
    if (retval != SQLITE_OK) {
        BOOST_LOG_TRIVIAL(error) << "Database connection failed in InternetUsageStatistics!";
    }
    assert(retval == SQLITE_OK);
    BOOST_LOG_TRIVIAL(info) << "SQLite connection to " << dbPath << " successfully established in InternetUsageStatistics!";

    std::string queryString(
        " SELECT value FROM unbroadbandstats as un,"
        "     rel_country_to_un as rel"
        " WHERE un.country_or_area = rel.country_or_area"
        " AND rel.country = ?"
        " GROUP BY un.country_or_area"
        " HAVING max(year)");

    assert(sqlite3_prepare_v2(_sqliteDB, queryString.c_str(), queryString.size(), &_stmt, NULL) == SQLITE_OK);
}

// returns percent of population with Internet access in this country
// 0.0 if not found
double InternetUsageStatistics::operator[](std::string& countryName) {
    assert(sqlite3_bind_text(_stmt, 1, countryName.c_str(), countryName.size(), NULL) == SQLITE_OK);
    int retval = sqlite3_step(_stmt);
    double result = 0.0;
    if (retval == SQLITE_ROW)
        result = sqlite3_column_double(_stmt, 0);
    sqlite3_reset(_stmt);
    return result;
}
