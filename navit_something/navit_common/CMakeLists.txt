cmake_minimum_required(VERSION 3.0.2)
project(navit_common)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PROTOBUF_INCLUDE_DIRS}
)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        geodesy
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS include ${OpenCV_INCLUDE_DIRS}
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp  navit_msgs pluginlib rviz
        #  DEPENDS system_lib
)
add_library(${PROJECT_NAME}  
        src/geometry_algorithms.cpp
        src/coordinate_trans.cpp
        )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${BOOST_LIBRARIES}
  ${OpenCV_LIBS}
 ${catkin_LIBRARIES}
)

# add_custom_command(TARGET navit_proto_lib
#   POST_BUILD
#   COMMAND ${CMAKE_COMMAND} -E make_directory ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/
#   COMMAND ${CMAKE_COMMAND} -E copy_if_different
#   ${CMAKE_CURRENT_BINARY_DIR}/*.pb.h
#   ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/
#   COMMAND ${CMAKE_COMMAND} -E copy_if_different
#   ${CMAKE_CURRENT_BINARY_DIR}/*.pb.cc
#   ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/
# )

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

