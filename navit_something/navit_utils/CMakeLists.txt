cmake_minimum_required(VERSION 3.0.2)
project(navit_utils)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  navit_msgs
  rviz
  pluginlib
  message_generation
  navit_common
)
add_service_files(
  FILES
  GetSelection.srv
  GetPolylines.srv
  GetPoints.srv
)
add_message_files(
  FILES
  PolygonArrayStamped.msg
  Polyline.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
)


# find_package(pybind11 REQUIRED)
# find_package(Python REQUIRED)

find_package(Qt5Widgets) 
include_directories(${Qt5Widgets_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
## Declare a C++ library

add_executable(${PROJECT_NAME}_node src/node_monitor_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node  ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_geometry_recoorder_node src/geometry_recorder.cpp)
add_dependencies(${PROJECT_NAME}_geometry_recoorder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_geometry_recoorder_node  ${catkin_LIBRARIES})

#add_library(polygon_array src/rviz_display_polygon_array.cpp)
#add_dependencies(polygon_array ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(polygon_array  ${catkin_LIBRARIES})

# qt5_wrap_cpp(MOC_FILES_POLYGON
#   include/navit_utils/rviz_polygon_selection_tool.h
# )
# add_library(rviz_polygon_plugin SHARED 
#   ${MOC_FILES_POLYGON}
#   src/rviz_polygon_selection_tool.cpp)

# add_dependencies(rviz_polygon_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(rviz_polygon_plugin ${catkin_LIBRARIES} Qt5::Widgets ${QT_LIBRARIES})

# qt5_wrap_cpp(MOC_FILES_LINE
#   include/navit_utils/rviz_connection_line.h
# )
# add_library(rviz_line_plugin SHARED 
#   ${MOC_FILES_LINE}
#   src/rviz_connection_line.cpp)

# add_dependencies(rviz_line_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(rviz_line_plugin ${catkin_LIBRARIES} Qt5::Widgets ${QT_LIBRARIES})

# qt5_wrap_cpp(MOC_FILES_POINTS
#   include/navit_utils/rviz_points_selection_tool.h
# )
# add_library(rviz_point_plugin SHARED 
#   ${MOC_FILES_POINTS}
#   src/rviz_points_selection_tool.cpp)

# add_dependencies(rviz_point_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(rviz_point_plugin ${catkin_LIBRARIES} Qt5::Widgets ${QT_LIBRARIES})

add_executable(save_polygon_to_json_node 
    src/save_polygon_to_json.cpp
)

add_dependencies(save_polygon_to_json_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(save_polygon_to_json_node ${catkin_LIBRARIES} jsoncpp)

add_executable(convert_geometry_to_gridmap_node
  src/convert_geometry_to_gridmap.cpp
)
add_dependencies(convert_geometry_to_gridmap_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(convert_geometry_to_gridmap_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(tf_node src/convert_pose_to_tf_node.cpp)
add_dependencies(tf_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_node ${catkin_LIBRARIES})
## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS ${PROJECT_NAME}_node
                ${PROJECT_NAME}_geometry_recoorder_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
    config
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
