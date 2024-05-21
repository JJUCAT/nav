cmake_minimum_required(VERSION 3.0.2)
project(nem_local_planner)
add_compile_options(-std=c++11 -g)

# ignore all the warnings about unused parameters
add_compile_options(-w)

find_package(catkin REQUIRED
        COMPONENTS
        tf2
        angles
        roscpp
        tf2_ros
        navit_core
        nav_msgs
        pluginlib
        navit_costmap
        sensor_msgs
        cmake_modules
        geometry_msgs
        tf2_geometry_msgs
        teb_local_planner
        message_generation
        dynamic_reconfigure
        nem_global_planner
        actionlib
        actionlib_msgs
        )

find_package(Eigen3 REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        tf2
        roscpp
        tf2_ros
        nav_msgs
        pluginlib
        sensor_msgs
        teb_local_planner
        dynamic_reconfigure
)

check_include_file(sys/time.h HAVE_SYS_TIME_H)
if (HAVE_SYS_TIME_H)
    add_definitions(-DHAVE_SYS_TIME_H)
endif (HAVE_SYS_TIME_H)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -rdynamic -std=c++11 -Wno-deprecated")

add_library(${PROJECT_NAME}
        src/local_supporter.cpp
        src/nem_local_planner_ros.cpp
        src/pid_control.cpp
        )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(FILES nem_local_planner_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )
