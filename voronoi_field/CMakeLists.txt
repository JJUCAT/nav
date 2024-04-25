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
    costmap_2d
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(
  include
  3rdparty
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
      3rdparty
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

# 第三方库
# 第三方 dynamic voronoi
add_library(lib3rd_dynamic_voronoi
  3rdparty/dynamic_voronoi/dynamicvoronoi.cpp
)

# 第三方库接口
add_library(nanoflann_port
  src/nanoflann_port.cpp)
add_dependencies(nanoflann_port
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(nanoflann_port
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)


add_library(dynamic_voronoi_port
  src/dynamic_voronoi_port.cpp)
add_dependencies(dynamic_voronoi_port
  lib3rd_dynamic_voronoi
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamic_voronoi_port
  lib3rd_dynamic_voronoi
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)


# 生成一个 lib 需要哪些文件
add_library(voronoi_field_layer
  src/voronoi_field.cpp
)
# lib 依赖哪些文件，在生成 lib 之前需要先编译的文件
add_dependencies(voronoi_field_layer
  nanoflann_port
  dynamic_voronoi_port
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
# lib 链接需要的文件/库，主要是调用的函数所在的库
target_link_libraries(voronoi_field_layer
  nanoflann_port
  dynamic_voronoi_port
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/voronoi_field_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${CERES_LIBRARIES}
  ${catkin_LIBRARIES}
)

# gtest 参考 costmap2d 的配置

install(TARGETS
    voronoi_field_layer
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
