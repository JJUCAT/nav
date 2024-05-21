cmake_minimum_required(VERSION 3.0.2)
project(dock_visual_perception)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  std_msgs
  cv_bridge
  sensor_msgs
  tf
  image_transport
  geometry_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
  resource_retriever
  pluginlib
  navit_auto_dock
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TinyXML REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
        # ${PROJECT_NAME}_msgs
        cv_bridge
        dynamic_reconfigure
        geometry_msgs
        image_transport
        message_runtime
        pcl_conversions
        pcl_ros
        resource_retriever
        roscpp
        sensor_msgs
        std_msgs
        tf
        tf2
        visualization_msgs
        navit_auto_dock
)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${TinyXML_INCLUDE_DIRS}
)
include_directories(/usr/include/eigen3/)
include_directories(include/dock_visual_perception/)
include_directories(/opt/ros/noetic/include/tf)
include_directories(/usr/include/)

add_library(${PROJECT_NAME}
    src/Ar.cpp
    src/Bitset.cpp
    src/Camera.cpp
    src/ConnectedComponents.cpp
    src/Draw.cpp
    src/FileFormatUtils.cpp
    src/kinect_filtering.cpp
    src/Line.cpp
    src/Marker.cpp
    src/MarkerDetector.cpp
    src/VisualPerception.cpp
    src/Pose.cpp
    src/Rotation.cpp
    src/Util.cpp

)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${TinyXML_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(main src/main.cpp)
target_link_libraries(main ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME} main
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
install(FILES
   dock_visual_perception_plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
