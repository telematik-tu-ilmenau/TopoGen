# Find the COIN-OR::Lemon includes and library
# This module defines
#  COIN_LEMON_INCLUDE_DIR: where to find lemon/
#  COIN_LEMON_LIBRARIES: the libraries needed to use lemon
#  COIN_LEMON_FOUND: if false, do not try to use lemon

IF(COIN_LEMON_INCLUDE_DIR AND COIN_LEMON_LIBRARIES)
    SET(COIN_LEMON_FOUND TRUE)
ELSE(COIN_LEMON_INCLUDE_DIR AND COIN_LEMON_LIBRARIES)
    FIND_PATH(COIN_LEMON_INCLUDE_DIR lemon/grosso_locatelli_pullan_mc.h
        /usr/include
        /usr/local/include
    )
    FIND_LIBRARY(COIN_LEMON_LIBRARIES NAMES emon libemon
        PATHS
        /usr/lib
        /usr/local/lib
        /usr/lib64
        /usr/local/lib64
    )

    IF(COIN_LEMON_INCLUDE_DIR AND COIN_LEMON_LIBRARIES)
        SET(COIN_LEMON_FOUND TRUE)
        MESSAGE(STATUS "Found COIN-OR::Lemon: ${COIN_LEMON_INCLUDE_DIR}, ${COIN_LEMON_LIBRARIES}")
    ELSE(COIN_LEMON_INCLUDE_DIR AND COIN_LEMON_LIBRARIES)
        SET(COIN_LEMON_FOUND FALSE)
        MESSAGE(STATUS "COIN-OR::Lemon not found.")
    ENDIF(COIN_LEMON_INCLUDE_DIR AND COIN_LEMON_LIBRARIES)

    MARK_AS_ADVANCED(COIN_LEMON_INCLUDE_DIR COIN_LEMON_LIBRARIES)
ENDIF(COIN_LEMON_INCLUDE_DIR AND COIN_LEMON_LIBRARIES)
