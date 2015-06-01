cmake_minimum_required(VERSION 2.8)
project(topoGen)

# The version number.
set (topoGen_VERSION_MAJOR 1)
set (topoGen_VERSION_MINOR 0)

#
# CONFIGURATION VARIABLES
#
set(CMAKE_BUILD_TYPE debug)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

set(TOOL_NAME topoGen)
if (CMAKE_BUILD_TYPE STREQUAL debug)
  set(PREFIX ${PROJECT_SOURCE_DIR})
else(CMAKE_BUILD_TYPE STREQUAL debug)
  set(PREFIX ${PROJECT_SOURCE_DIR})
endif(CMAKE_BUILD_TYPE STREQUAL debug)

set(DATAROOTDIR ${PREFIX}/share)
set(BINDIR ${PREFIX}/bin)
set(SRCDIR ${PREFIX}/src)
set(CONFIGFILE ${DATAROOTDIR}/${PROJECT_NAME}/config.json)

# set configuration variables
configure_file(src/config/Defines.hpp.cmake
               src/config/Defines.hpp)

#
# COMPILER OPTIONS
#
set(CMAKE_CXX_FLAGS "-g -Wall -Wextra -march=native -O3 -std=c++11")
include_directories("${SRCDIR}" "${CMAKE_BINARY_DIR}/src" "/usr/local/include")
link_directories("/usr/local/lib")

#
# ADD SOURCES AND LINK LIBRARIES
#

add_subdirectory(src)
