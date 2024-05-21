cmake_minimum_required(VERSION 3.0.2)
project(fcpp_ros)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  fcpp_algorithm
  navit_msgs
  pluginlib
  navit_core
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fcpp_ros
  CATKIN_DEPENDS roscpp navit_msgs fcpp_algorithm navit_core
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
 #src/FcppNode.cpp
 src/TopologyPlan.cpp
 src/FcppAction.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
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
  fcpp.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_fcpp_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
