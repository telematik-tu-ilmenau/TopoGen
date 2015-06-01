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

#include "Config.hpp"

#include "PredefinedValues.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <utility>

Config::Config() {
    std::ifstream configFile(PredefinedValues::configfile());
    configFile >> _config;
}

Config::Config(std::string fileName) {
    std::ifstream configFile(fileName);
    configFile >> _config;
}

Config_Ptr Config::subConfig(std::string propertyName) {
    std::pair<Json::Value, std::string> node = getSubNode(propertyName);
    Json::Value& nd = node.first;
    std::string& key = node.second;
    return Config_Ptr(new Config(nd[key]));
}

Config::Config(Json::Value node) : _config(node) {
}

Json::Value Config::getSubValue(std::string propertyName) {
    std::pair<Json::Value, std::string> node = getSubNode(propertyName);
    Json::Value& nd = node.first;
    std::string& key = node.second;
    return nd[key];
}

template <>
std::string Config::get<std::string>(std::string propertyName) {
    return getSubValue(propertyName).asString();
}

template <>
double Config::get<double>(std::string propertyName) {
    return getSubValue(propertyName).asDouble();
}

template <>
int Config::get<int>(std::string propertyName) {
    return getSubValue(propertyName).asInt();
}

template <>
bool Config::get<bool>(std::string propertyName) {
    return getSubValue(propertyName).asBool();
}

template <>
unsigned int Config::get<unsigned int>(std::string propertyName) {
    return static_cast<unsigned int>(getSubValue(propertyName).asInt());
}

std::pair<Json::Value, std::string> Config::getSubNode(std::string propertyName) {
    Json::Value node = _config;

    std::vector<std::string> splittedNames(splitPropertyNameBy(propertyName));
    for (size_t i = 0; i < splittedNames.size() - 1; ++i) {
        node = node[splittedNames[i]];
    }

    return std::make_pair(node, splittedNames[splittedNames.size() - 1]);
}

std::vector<std::string> Config::splitPropertyNameBy(std::string propertyName, char delimiter) {
    std::vector<std::string> vec;

    std::istringstream tokens(propertyName);
    std::string currentToken;
    while (std::getline(tokens, currentToken, delimiter))
        vec.push_back(currentToken);

    return vec;
}
