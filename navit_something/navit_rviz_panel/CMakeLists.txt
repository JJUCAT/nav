cmake_minimum_required(VERSION 3.0.2)
project(navit_rviz_panel)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  navit_msgs
  rviz
  pluginlib
  actionlib
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
 INCLUDE_DIRS include
 LIBRARIES navit_rviz_panel navit_rviz_panel_proto navit_common
 CATKIN_DEPENDS roscpp navit_msgs
)
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
message(STATUS "PROTOBUF_INCLUDE_DIRS: ${PROTOBUF_INCLUDE_DIRS}")

file(GLOB ProtoFiles proto/*.proto)
protobuf_generate_cpp(ProtoSources ProtoHeaders ${ProtoFiles})
add_library(navit_rviz_panel_proto SHARED ${ProtoSources})

target_link_libraries(navit_rviz_panel_proto ${PROTOBUF_LIBRARIES})

find_package(Qt5Widgets) 
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CATKIN_DEVEL_PREFIX}/include
)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

# I prefer the Qt signals and slots to avoid defining "emit", "slots",
# etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here
# e.g. http://muddyazian.blogspot.de/2012/04/getting-qt-app-working-with-boost-using.html
add_definitions(-DQT_NO_KEYWORDS)

qt5_wrap_cpp(MOC_FILES_POLYGON
  include/navit_rviz_panel/rviz_polygon_selection_tool.h
)
add_library(rviz_polygon_plugin SHARED 
  ${MOC_FILES_POLYGON}
  src/rviz_polygon_selection_tool.cpp)

add_dependencies(rviz_polygon_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rviz_polygon_plugin ${catkin_LIBRARIES} Qt5::Widgets ${QT_LIBRARIES})

qt5_wrap_cpp(MOC_FILES_LINE
  include/navit_rviz_panel/rviz_connection_line.h
)
add_library(rviz_line_plugin SHARED 
  ${MOC_FILES_LINE}
  src/rviz_connection_line.cpp)

add_dependencies(rviz_line_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rviz_line_plugin ${catkin_LIBRARIES} Qt5::Widgets ${QT_LIBRARIES})

qt5_wrap_cpp(MOC_FILES_POINTS
  include/navit_rviz_panel/rviz_points_selection_tool.h
)
add_library(rviz_point_plugin SHARED 
  ${MOC_FILES_POINTS}
  src/rviz_points_selection_tool.cpp)

add_dependencies(rviz_point_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rviz_point_plugin ${catkin_LIBRARIES} Qt5::Widgets ${QT_LIBRARIES})

qt5_wrap_cpp(MOC_NAVIT_FILES_POINTS
  include/navit_rviz_panel/rviz_point_with_line_tool.h
)
add_library(navit_rviz_point_with_line_plugin SHARED 
  ${MOC_NAVIT_FILES_POINTS}
  src/rviz_point_with_line_tool.cpp)

add_dependencies(navit_rviz_point_with_line_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navit_rviz_point_with_line_plugin ${catkin_LIBRARIES} Qt5::Widgets ${QT_LIBRARIES} navit_rviz_panel_proto)

# Define source file
set(${PROJECT_NAME}_SRCS
  src/rviz_panel.cpp
)

# Define header file
set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/rviz_panel.h
)

# Define ui file
set(${PROJECT_NAME}_UIS
  ui/custom_panel.ui
)

# Create header from ui file (uic)
if(rviz_QT_VERSION VERSION_LESS "5")
    message(STATUS "Generate header for ui with rviz_QT_VERSION: ${rviz_QT_VERSION}")
    qt4_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})
    qt4_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})
else()
    message(STATUS "Generate header for ui with rviz_QT_VERSION: ${rviz_QT_VERSION}")
    qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})
    qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})
endif()

## Add library is needed in order to generate the header file from ui file.
add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_UIS_H}
  ${${PROJECT_NAME}_MOCS}
  
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  jsoncpp
)

add_executable(${PROJECT_NAME}_with_panel_test_node 
            test/fcpp_server_test.cpp)
add_dependencies(${PROJECT_NAME}_with_panel_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_with_panel_test_node ${catkin_LIBRARIES} navit_rviz_panel_proto)

# install(TARGETS ${PROJECT_NAME}_node
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# ## Mark libraries for installation
# ## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html

# ## Mark cpp header files for installation
install(TARGETS ${PROJECT_NAME}
  navit_rviz_panel_proto
  navit_rviz_point_with_line_plugin
  rviz_point_plugin
  rviz_line_plugin
  rviz_polygon_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

install(FILES display_plugins.xml rviz_panel.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)