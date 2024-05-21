cmake_minimum_required(VERSION 3.0.2)
project(visibility_graph_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  navit_msgs
  navit_core
  navit_router
  path_smoother
  # message_generation
)
include_directories(
  include
  ${path_smoother}/include
  ${catkin_INCLUDE_DIRS}
)
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES visibility_graph_planner
  CATKIN_DEPENDS  navit_router roscpp std_msgs navit_msgs path_smoother navit_core
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


## Declare a C++ library
add_library(${PROJECT_NAME}
   src/visibility_graph_planner_ros.cpp
   src/visibility_graph_planner.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${BOOST_LIBRARIES}
  jsoncpp
)

add_executable(${PROJECT_NAME}_node
   src/visibility_graph_planner_ros.cpp
)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  jsoncpp
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  visibility_graph_planner.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
