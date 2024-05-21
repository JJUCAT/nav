cmake_minimum_required(VERSION 3.0.2)
project(winnie_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
add_compile_options(-Wall)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        nodelet
        pluginlib
        std_msgs
        sensor_msgs
        tf2_ros
        navit_msgs
        # component_msgs
        # integrated_navigation_msg
        dynamic_reconfigure
        )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
        FILES
        IOStates.msg
        MotorCtrl.msg
        MotorInfo.msg
        MotorInfos.msg
)

## Generate services in the 'srv' folder
add_service_files(
        FILES
        SetIO.srv
        SetMotor.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs  # Or other packages containing msgs
        sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
#generate_dynamic_reconfigure_options(
#        cfg/WinnieLQR.cfg
#)

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
        #  INCLUDE_DIRS include
        LIBRARIES winnie_base nodelet_winnie_base winnie_base_node winnie_device_rtk winnie_device_agv
        winnie_device_imu winnie_device_odom winnie_device_motor winnie_device_io winnie_device_battery
        CATKIN_DEPENDS roscpp nodelet pluginlib tf2_ros navit_msgs dynamic_reconfigure
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
#add_library(${PROJECT_NAME} src/nodelet_winnie_base.cpp)
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} gnss_parser2)
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} gnss_parser2)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#add_executable(winnie_base_node src/winnie_base_node.cpp)
#target_link_libraries(winnie_base_node ${PROJECT_NAME} ${catkin_LIBRARIES})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

add_subdirectory(winnie_sdk)

## plugin device.
add_subdirectory(plugins/GnssParserLib)
add_library(winnie_device_rtk plugins/device_rtk.cpp)
add_dependencies(winnie_device_rtk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} gnss_parser2 winnie_sdk)
target_link_libraries(winnie_device_rtk ${catkin_LIBRARIES} gnss_parser2 winnie_sdk)

add_library(winnie_device_agv plugins/device_agv.cpp)
add_dependencies(winnie_device_agv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} winnie_sdk)
target_link_libraries(winnie_device_agv ${catkin_LIBRARIES} winnie_sdk)

add_library(winnie_device_imu plugins/device_imu.cpp)
add_dependencies(winnie_device_imu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} winnie_sdk)
target_link_libraries(winnie_device_imu ${catkin_LIBRARIES} winnie_sdk)

add_library(winnie_device_odom plugins/device_odom.cpp)
add_dependencies(winnie_device_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} winnie_sdk)
target_link_libraries(winnie_device_odom ${catkin_LIBRARIES} winnie_sdk)

add_library(winnie_device_motor plugins/device_motor.cpp)
add_dependencies(winnie_device_motor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} winnie_sdk)
target_link_libraries(winnie_device_motor ${catkin_LIBRARIES} winnie_sdk)

add_library(winnie_device_io plugins/device_io.cpp)
add_dependencies(winnie_device_io ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} winnie_sdk)
target_link_libraries(winnie_device_io ${catkin_LIBRARIES} winnie_sdk)

add_library(winnie_device_battery plugins/device_battery.cpp)
add_dependencies(winnie_device_battery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} winnie_sdk)
target_link_libraries(winnie_device_battery ${catkin_LIBRARIES} winnie_sdk)


## Declare a C++ library
add_library(winnie_base src/winnie_base.cpp)
add_dependencies(winnie_base ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} winnie_sdk)
target_link_libraries(winnie_base ${catkin_LIBRARIES} winnie_sdk)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

add_library(nodelet_winnie_base src/nodelet_winnie_base.cpp)
add_dependencies(nodelet_winnie_base winnie_base ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} winnie_sdk)
target_link_libraries(nodelet_winnie_base winnie_base ${catkin_LIBRARIES} winnie_sdk)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(winnie_base_node src/winnie_base_node.cpp)
target_link_libraries(winnie_base_node winnie_base ${catkin_LIBRARIES} winnie_sdk)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
#install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
### Mark libraries for installation
### See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
### Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)
#
### Mark other files for installation (e.g. launch and bag files, etc.)
#install(DIRECTORY
#    config
#    launch
#    scripts
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

install(DIRECTORY
        launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(FILES winnie_base.xml winnie_base_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(TARGETS winnie_sdk gnss_parser2 winnie_base nodelet_winnie_base winnie_base_node winnie_base_node winnie_device_rtk winnie_device_agv
        winnie_device_imu winnie_device_odom winnie_device_motor winnie_device_io winnie_device_battery
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_winnie_base.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
