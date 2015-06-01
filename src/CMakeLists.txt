#
# ADD SOURCES
#

FILE(GLOB_RECURSE topoSources *.cpp)

add_executable(topoGen ${topoSources})


#
# LINKER OPTIONS
#

find_package(Boost REQUIRED COMPONENTS program_options system thread)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

find_package(CGAL REQUIRED)
include_directories(${CGAL_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ${CGAL_LIBRARIES})

find_package(GMP REQUIRED)
include_directories(${GMP_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ${GMP_LIBRARIES})

find_package(JsonCpp REQUIRED)
include_directories(${JSONCPP_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${JSONCPP_LIBRARIES})

find_package(Lemon REQUIRED)
include_directories(${COIN_LEMON_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ${COIN_LEMON_LIBRARIES})

find_package(SQLite3 REQUIRED)
include_directories(${SQLITE3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${SQLITE3_LIBRARIES})

find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})


#
# INSTALLATION OPTIONS
#

install(TARGETS topoGen
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
