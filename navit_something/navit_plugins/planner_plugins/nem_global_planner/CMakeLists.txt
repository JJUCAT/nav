cmake_minimum_required(VERSION 3.0.2)
project(nem_global_planner)
add_compile_options(-std=c++11 -g)

# ignore warnings
add_compile_options(-w)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        tf2_ros
        navit_core
        pluginlib
        geometry_msgs
        tf2_geometry_msgs
        dynamic_reconfigure
        )


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        navfn
        roscpp
        tf2_ros
        navit_core
        nav_msgs
        pluginlib
        navit_costmap
        geometry_msgs
        dynamic_reconfigure
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -rdynamic -std=c++11 -Wno-deprecated")

add_library(${PROJECT_NAME}
        src/pose.cpp
        src/cedge.cpp
        src/cnode.cpp
        src/nem_global_planner_ros.cpp
        )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} jsoncpp)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE)

install(FILES nem_global_planner_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
