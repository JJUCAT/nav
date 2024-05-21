cmake_minimum_required(VERSION 3.0.2)
project(path_smoother)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  navit_core
  pluginlib
  roscpp
  roslib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_smoother
# CATKIN_DEPENDS costmap_2d gemoetry_msgs nav_msgs navit_collision_checker navit_core navit_msgs pluginlib roscpp std_msgs tf2_ros #  DEPENDS system_lib
)
find_package(GSL REQUIRED)
set(GSL_DIR /usr/include/gsl)
message(STATUS "GSL_DIR: ${GSL_DIR}")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
message(STATUS "PROTOBUF_INCLUDE_DIRS: ${PROTOBUF_INCLUDE_DIRS}")

file(GLOB ProtoFiles proto/*.proto)
protobuf_generate_cpp(ProtoSources ProtoHeaders ${ProtoFiles})
add_library(navit_proto SHARED ${ProtoSources})

target_link_libraries(navit_proto ${PROTOBUF_LIBRARIES})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_library(${PROJECT_NAME}
   src/path_spline.cpp
)

target_link_libraries(${PROJECT_NAME} ${GSL_LIBRARIES} ${PROTOBUF_LIBRARIES} navit_proto)
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
                navit_proto
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES path_smoother.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
