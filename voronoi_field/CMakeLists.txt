cmake_minimum_required(VERSION 3.0.2)
project(voronoi_field)

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    dynamic_reconfigure
    geometry_msgs
    map_msgs
    nav_msgs
    pluginlib
    roscpp
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

# messages
# add_message_files(
#     DIRECTORY msg
#     FILES
#     VoxelGrid.msg
# )

# generate_messages(
#     DEPENDENCIES
#         std_msgs
#         geometry_msgs
#         map_msgs
# )

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/VoronoiFieldPlugin.cfg
)

catkin_package(
  INCLUDE_DIRS
      include
      ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES voronoi_field
  CATKIN_DEPENDS
      costmap_2d
      dynamic_reconfigure
      geometry_msgs
      map_msgs
      nav_msgs
      pluginlib
      roscpp
      std_msgs
      tf2_ros
      visualization_msgs
  DEPENDS
      EIGEN3
      Boost
)

add_library(voronoi_field
  src/voronoi_field.cpp
)
add_dependencies(voronoi_field ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(voronoi_field
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

# gtest 参考 costmap2d 的配置

install(TARGETS
    voronoi_field
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
