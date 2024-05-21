cmake_minimum_required(VERSION 3.0.2)
project(ipa_voronoi_explorator)

# build with c++11
add_compile_options(-std=c++11)

set(catkin_RUN_PACKAGES
        eigen_conversions
        geometry_msgs
        nav_msgs
        roscpp
        roslib
        std_msgs
        navit_core
        navit_utils
        navit_msgs
        )

set(catkin_BUILD_PACKAGES
        ${catkin_RUN_PACKAGES}
        dynamic_reconfigure
        )

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        ${catkin_BUILD_PACKAGES}
        )

generate_dynamic_reconfigure_options(
        cfg/VoronoiExplorator.cfg
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system chrono thread)

find_package(Eigen3)
if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
# important: common/include needs to be here if you have such a directory
catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        ${catkin_RUN_PACKAGES}
        DEPENDS
        OpenCV
        Boost
)


###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
# important: common/include also needs to be here if you have it, order of including the coinor-packages relevant, not working when including in wrong order
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

### package library
add_library(${PROJECT_NAME}
        # voronoi_explorator_ros
        src/voronoi_explorator_ros.cpp
        # ipa_room_exploration
        src/ipa_room_exploration/room_rotator.cpp
        # ipa_building_navigation
        src/ipa_building_navigation/A_star_pathplanner.cpp
        src/ipa_building_navigation/node.cpp
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        )
add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        )

add_executable(voronoi_explorator_planner_test test/voronoi_explorator_planner_test.cpp)
target_link_libraries(voronoi_explorator_planner_test ${PROJECT_NAME})

add_executable(voronoi_explorator_test test/voronoi_explorator_test.cpp)
target_link_libraries(voronoi_explorator_test ${PROJECT_NAME})

#############
## Install ##
#############
# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

#uncomment this if you have a common-directory with header-files
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )

#install(DIRECTORY scripts
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#	PATTERN ".svn" EXCLUDE
#)

# uncomment this if you have launch files
#install(DIRECTORY launch
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#        PATTERN ".svn" EXCLUDE
#        )

#install(DIRECTORY common/files
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common
#	PATTERN ".svn" EXCLUDE
#)

install(FILES
        ipa_voronoi_explorator.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
