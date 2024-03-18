project(dynamicEDT3D)
cmake_minimum_required(VERSION 2.6)

SET(CMAKE_BUILD_TYPE Release)

SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

set(octomap_DIR "" CACHE PATH "Set this to the octomap base directory if finding the package fails. Please use the latest trunk version of octomap from  https://octomap.svn.sourceforge.net/svnroot/octomap/trunk")

find_package(octomap HINTS ${octomap_DIR}) 
IF(NOT octomap_FOUND)
  MESSAGE( FATAL_ERROR "Could not find the octomap package. Please set the variable octomap_DIR")
ENDIF(NOT octomap_FOUND)
INCLUDE_DIRECTORIES(${OCTOMAP_INCLUDE_DIRS})
LINK_DIRECTORIES(${OCTOMAP_LIBRARY_DIRS})

# COMPILER FLAGS
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -ggdb")

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

ADD_SUBDIRECTORY(src)


# Documentation 
FIND_PACKAGE(Doxygen)
IF(DOXYGEN_FOUND)
  ADD_CUSTOM_TARGET(docs ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/dynamicEDT3D.dox
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Generating documentation (Doxygen)...")
ENDIF(DOXYGEN_FOUND)
