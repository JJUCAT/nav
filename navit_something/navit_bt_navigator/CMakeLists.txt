cmake_minimum_required(VERSION 3.0.2)
project(navit_bt_navigator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++1z)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        actionlib
        behaviortree_cpp_v3
        geometry_msgs
        navit_msgs
        roscpp
        rospy
        roslib
        std_msgs
        tf2_ros
        navit_bt_nodes
        backward_ros
        )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})

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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   navit_msgs#   std_msgs
# )

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
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
        INCLUDE_DIRS include
        LIBRARIES navit_bt_navigator
        CATKIN_DEPENDS actionlib geometry_msgs navit_msgs roscpp rospy std_msgs tf2_ros behaviortree_cpp_v3 backward_ros
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
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/navit_bt_navigator.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/navit_bt_navigator_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

# add_executable(way_points_navigator_action_server_node src/way_points_navigator_action_server_node.cpp)
# add_dependencies(way_points_navigator_action_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(way_points_navigator_action_server_node ${catkin_LIBRARIES})

# add_executable(way_points_navigator_action_client_node src/way_points_navigator_action_client_node.cpp)
# add_dependencies(way_points_navigator_action_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(way_points_navigator_action_client_node ${catkin_LIBRARIES})

# add_executable(simple_navigator_action_server_node src/simple_navigator_action_server_node.cpp)
# add_dependencies(simple_navigator_action_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(simple_navigator_action_server_node ${catkin_LIBRARIES})

# add_executable(simple_navigator_action_client_node src/simple_navigator_action_client_node.cpp)
# add_dependencies(simple_navigator_action_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(simple_navigator_action_client_node ${catkin_LIBRARIES})

add_executable(simple_navigator src/simple_navigator.cpp)
add_dependencies(simple_navigator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_navigator ${catkin_LIBRARIES})

# for j110
add_executable(j110_navigator src/j110_navigator.cpp)
add_dependencies(j110_navigator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(j110_navigator ${catkin_LIBRARIES})

# add_executable(way_points_navigator src/way_points_navigator.cpp)
# add_dependencies(way_points_navigator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(way_points_navigator ${catkin_LIBRARIES})

# add_executable(auto_dock_sub_navigator src/auto_dock_sub_navigator.cpp)
# add_dependencies(auto_dock_sub_navigator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(auto_dock_sub_navigator ${catkin_LIBRARIES})

# navit_bt_runner
add_library(navit_bt_runner src/navit_bt_runner.cpp)
add_dependencies(navit_bt_runner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navit_bt_runner ${catkin_LIBRARIES})

# # move_to_pose_action_server
# add_executable(move_to_pose_action_server src/move_to_pose_action_server.cpp)
# add_dependencies(move_to_pose_action_server navit_bt_runner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(move_to_pose_action_server navit_bt_runner ${catkin_LIBRARIES})

# # move_through_poses_action_server
add_executable(move_through_poses_action_server src/move_through_poses_action_server.cpp)
add_dependencies(move_through_poses_action_server navit_bt_runner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_through_poses_action_server navit_bt_runner ${catkin_LIBRARIES})

# back to reference line server
add_executable(back_to_reference_line_action_server_node src/back_to_reference_line_action_server.cpp)
add_dependencies(back_to_reference_line_action_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(back_to_reference_line_action_server_node navit_bt_runner ${catkin_LIBRARIES})

# add_executable(j110_auto_dock_node src/j110/auto_dock_server.cpp)
# add_dependencies(j110_auto_dock_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(j110_auto_dock_node navit_bt_runner ${catkin_LIBRARIES})

# add_library(j110_bt_server_lib
#          src/j110/auto_dock_server.cpp)
# target_link_libraries(j110_bt_server_lib navit_bt_runner ${catkin_LIBRARIES})

# add_executable(j110_bt_server_node src/j110/j110_bt_server_node.cpp)
# add_dependencies(j110_bt_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(j110_bt_server_node navit_bt_runner ${catkin_LIBRARIES})

add_executable(navigate_to_pose_node src/j110/navigate_to_pose_action_server.cpp)
add_dependencies(navigate_to_pose_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigate_to_pose_node navit_bt_runner ${catkin_LIBRARIES})

add_executable(navigate_through_path_action_server_node src/j110/navigate_through_path_action_server.cpp)
add_dependencies(navigate_through_path_action_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigate_through_path_action_server_node navit_bt_runner ${catkin_LIBRARIES})

add_executable(navigate_compute_coverage_path_action_server_node src/j110/navigate_compute_coverage_path_action_server.cpp)
add_dependencies(navigate_compute_coverage_path_action_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigate_compute_coverage_path_action_server_node navit_bt_runner ${catkin_LIBRARIES})

add_executable(navigate_auto_dock_action_server_node src/j110/navigate_auto_dock_action_server.cpp)
add_dependencies(navigate_auto_dock_action_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigate_auto_dock_action_server_node navit_bt_runner ${catkin_LIBRARIES})
#############
## Install ##
#############
install(PROGRAMS
        scripts/circle_viapoints.py
        scripts/cmd_vel_to_ackermann_drive.py
        scripts/export_to_mat.py
        scripts/export_to_svg.py
        scripts/move_obstacle.py
        scripts/publish_dynamic_obstacle.py
        scripts/publish_ground_truth_obstacles.py
        scripts/publish_test_obstacles.py
        scripts/publish_viapoints.py
        scripts/square_viapoints.py
        scripts/visualize_obstacle_velocity_profile.py
        scripts/visualize_velocity_profile.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS
        # way_points_navigator_action_server_node
        # way_points_navigator_action_client_node
        # simple_navigator_action_server_node
        # simple_navigator_action_client_node
        simple_navigator
        # way_points_navigator
        # auto_dock_sub_navigator
        back_to_reference_line_action_server_node
        navigate_through_path_action_server_node
        navigate_to_pose_node
        navigate_compute_coverage_path_action_server_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
install(TARGETS navit_bt_runner
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )
## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )


install(DIRECTORY
        behavior_trees
        cfg
        doc
        launch
        rviz
        maps
        stage
        scripts
        stage
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_navit_bt_navigator.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
