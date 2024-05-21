cmake_minimum_required(VERSION 2.8.3)
project(vanjee_lidar)

add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)

set( CMAKE_BUILD_TYPE Release )

find_package(catkin REQUIRED COMPONENTS
        angles
        nodelet
        pcl_ros
        roscpp
        sensor_msgs
        tf
        vanjee_msgs
        dynamic_reconfigure
        pcl_conversions
        )

find_package(yaml-cpp REQUIRED)
find_package(Boost COMPONENTS signals)
find_package(PkgConfig REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIR}
)


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES vanjee_lidar
        CATKIN_DEPENDS
        angles
        nodelet
        pcl_ros
        roscpp
        sensor_msgs
        tf
        vanjee_driver
        vanjee_msgs
        dynamic_reconfigure
        pcl_conversions
        DEPENDS
        Boost
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
)

add_library(vanjee_data src/lidar720.cpp src/lidar721.cpp  src/complementary_filter.cpp)
target_link_libraries(vanjee_data
        ${catkin_LIBRARIES}
        ${libpcap_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        )

add_library(vanjee_point src/convert.cc)
target_link_libraries(vanjee_point
        vanjee_data
        ${catkin_LIBRARIES}
        )


add_library(vanjee_pointcloud_nodelet src/cloud_nodelet.cc)
target_link_libraries(vanjee_pointcloud_nodelet vanjee_point
        ${catkin_LIBRARIES})

add_executable(vanjee_pointcloud_node src/cloud_node.cc)
target_link_libraries(vanjee_pointcloud_node
        vanjee_point
        ${catkin_LIBRARIES}
        ${libpcap_LIBRARIES})
add_dependencies(vanjee_pointcloud_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

install(TARGETS vanjee_pointcloud_node DESTINATION ./lib/vanjee_lidar)
install(DIRECTORY cfg launch data rviz_cfg DESTINATION ./share/vanjee_lidar)
install(DIRECTORY ../config DESTINATION ./share)
install(FILES nodelets.xml DESTINATION ./share/vanjee_lidar)
install(TARGETS vanjee_point vanjee_pointcloud_nodelet vanjee_data DESTINATION ./lib)

if(catkin_EXPORTED_TARGETS)
    add_dependencies(vanjee_data ${catkin_EXPORTED_TARGETS})
endif()


