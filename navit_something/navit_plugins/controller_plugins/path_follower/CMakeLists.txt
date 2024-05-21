cmake_minimum_required(VERSION 3.0.2)
project(path_follower)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  navit_core
  pluginlib
  roscpp
  roslib
  path_smoother
  navit_costmap
  navit_common
  smac_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_follower
  CATKIN_DEPENDS navit_costmap nav_msgs navit_collision_checker navit_core navit_msgs pluginlib roscpp std_msgs tf2_ros path_smoother navit_common smac_planner
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
message(STATUS "PROTOBUF_INCLUDE_DIRS: ${PROTOBUF_INCLUDE_DIRS}")

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/proto)
message(STATUS "PROTOBUF_INCLUDE_DIRS: ${CMAKE_CURRENT_BINARY_DIR}")

include_directories(${CMAKE_CURRENT_BINARY_DIR}/proto)

find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})
add_library(common_geometry2
        src/path_follower/geometry2.cpp
)

add_library(hybrid_astar
  src/path_planner/astar_util.cpp
  src/path_planner/astar_search.cpp
)
add_dependencies(hybrid_astar ${hybrid_astar_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(hybrid_astar_node
  src/path_planner/astar_search_node.cpp
)

add_dependencies(hybrid_astar_node ${hybrid_astar_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hybrid_astar_node ${PROTOBUF_LIBRARIES} common_geometry2 hybrid_astar ${catkin_LIBRARIES})

add_library(path_follower
  src/path_planner/spline.cpp
  src/pid_solver/pid_solver.cpp
  src/path_follower/path_follow_control.cpp
  src/path_follower/coordinate_convert.cpp
  src/path_follower/trajectory_view.cpp
  src/path_follower/path_matcher.cpp
  )

add_dependencies(path_follower ${path_follower_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_follower ${PROTOBUF_LIBRARIES} ${catkin_LIBRARIES} ${OMPL_LIBRARIES} path_follower_proto common_geometry2 hybrid_astar -ljsoncpp)

add_executable(${PROJECT_NAME}_node
  src/path_follow_local_planner_ros.cpp src/path_follower/path_matcher.cpp src/path_follower/path_matcher.cpp src/path_follower/trajectory_view.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME})

## Mark executables and/or libraries for installation
install(TARGETS path_follower common_geometry2 hybrid_astar
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS path_follower_node hybrid_astar_node
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
  path_follower.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# install(DIRECTORY
#   launch cfg scripts
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#   PATTERN ".svn" EXCLUDE
# )
