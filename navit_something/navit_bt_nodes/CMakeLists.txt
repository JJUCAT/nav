cmake_minimum_required(VERSION 3.16.3)
project(navit_bt_nodes)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  behaviortree_cpp_v3
  geometry_msgs
  pluginlib
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf2_ros
  navit_msgs
  rosbag
  navit_move_base
  navit_utils
  navit_rviz_panel
  tf
  navit_common
  app_chassis_control_msgs
  navit_costmap
  navit_collision_checker
  angles
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
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  actionlib
  actionlib_msgs
  behaviortree_cpp_v3
  geometry_msgs
  pluginlib
  roscpp
  rospy
  std_msgs
  tf2_ros
  navit_msgs
  navit_rviz_panel
  tf navit_common
  app_chassis_control_msgs
  navit_costmap
  navit_collision_checker
  #  DEPENDS system_lib
)

###########
## Build ##
###########

# 自定义函数：检查文件是否被修改
function(check_file_modified TARGET FILES)
    set(DEPENDENCIES)
    foreach (FILE ${FILES})
        set(MODIFIED_FILE "${CMAKE_CURRENT_BINARY_DIR}/${FILE}")

        # 检查文件是否被修改
        add_custom_command(OUTPUT ${MODIFIED_FILE}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FILE} ${MODIFIED_FILE}
                DEPENDS ${FILE})

        list(APPEND DEPENDENCIES ${MODIFIED_FILE})
    endforeach ()

    add_custom_target(${TARGET}_files_modified ALL DEPENDS ${DEPENDENCIES})
endfunction()


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${rosbag_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
        src/behavior_tree_engine.cpp
        src/navi_support.cpp
        )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${rosbag_LIBRARIES} jsoncpp yaml-cpp)


# 使用file命令获取指定文件夹下所有的.cpp文件路径
file(GLOB_RECURSE CPP_FILES "${CMAKE_CURRENT_SOURCE_DIR}/plugins/*.cpp")

# 输出相对路径及文件名
foreach (CPP_FILE ${CPP_FILES})
    # 获取相对路径
    # file(RELATIVE_PATH RELATIVE_PATH ${CMAKE_SOURCE_DIR} ${CPP_FILE})

    # 获取文件名
    get_filename_component(FILENAME ${CPP_FILE} NAME)
    # 去除文件名的后缀
    string(REGEX REPLACE "\\.[^.]*$" "" FILENAME_NO_EXT "${FILENAME}")

    set(lib_name navit_bt_nodes_${FILENAME_NO_EXT})
    add_library(${lib_name} SHARED ${CPP_FILE})
    add_dependencies(${lib_name} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(${lib_name} ${catkin_LIBRARIES})
    target_compile_definitions(${lib_name} PRIVATE BT_PLUGIN_EXPORT)

    list(APPEND plugin_libs ${lib_name})
endforeach ()


# task_receiver
add_subdirectory(task_receiver)
list(APPEND plugin_libs navit_bt_nodes_task_receiver)


################################################
################################################
################################################
# 构建带引号和逗号的每个元素的字符串
foreach (lib ${plugin_libs})
    string(APPEND plugin_libs_string "\"" ${lib} "\",\n")
endforeach ()

# 移除最后一个元素结尾的逗号
string(REGEX REPLACE ",\n$" "" plugin_libs_string "${plugin_libs_string}")

# 打印逗号分隔的字符串
message("plugin_lib_names:")
message(${plugin_libs_string})
# 以上打印复制到： navit_bt_navigator/simple_navigator.hpp 中 plugin_lib_names
################################################
################################################
################################################


## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME} ${plugin_libs}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )

install(FILES
        navit_tree_nodes.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
