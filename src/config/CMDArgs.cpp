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

#include "CMDArgs.hpp"

CMDArgs::CMDArgs(int argc, char** argv) : _desc("Allowed options"), _vm(), kmlOutput(false), graphOutput(false), jsonOutput(false), seed(), simNodesJSONPath(), jsonOutFile() {
    _desc.add_options()("help", "produce help message")("kml", po::value<bool>(&kmlOutput)->zero_tokens())(
        "json", po::value<bool>(&jsonOutput)->zero_tokens())("graph", po::value<bool>(&graphOutput)->zero_tokens())(
        "seed", po::value<std::string>(&seed)->default_value("run1"))(
        "jsonOutputFile", po::value<std::string>(&jsonOutFile)->default_value("graph.json"))(
        "simNodes", po::value<std::string>(&simNodesJSONPath)->default_value(""));

    po::store(po::parse_command_line(argc, argv, _desc), _vm);
    po::notify(_vm);
}

bool CMDArgs::kmlOutputEnabled() {
    return kmlOutput;
}

bool CMDArgs::graphOutputEnabled() {
    return graphOutput;
}

bool CMDArgs::jsonOutputEnabled() {
    return jsonOutput;
}

std::string CMDArgs::getSeed() {
    return seed;
}

std::string CMDArgs::simNodesJSONFile() {
    return simNodesJSONPath;
}

std::string CMDArgs::jsonOutputFile() {
    return jsonOutFile;
}
