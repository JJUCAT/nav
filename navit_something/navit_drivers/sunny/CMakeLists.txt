cmake_minimum_required(VERSION 3.14.5)
project(tof_dev_sdk_demo)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg
        message_generation sensor_msgs cv_bridge
        pcl_conversions pcl_msgs pcl_ros
        nodelet
        )


find_package(PCL REQUIRED)
find_package(OpenCV 4.0 REQUIRED)


## Declare ROS messages and services
#add_message_files(FILES XXX.msg)
#add_service_files(FILES XXX.srv)


## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)


## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)


## Build publisher and subscribe


message(STATUS " CMAKE_CURRENT_SOURCE_DIR=========== ${CMAKE_CURRENT_SOURCE_DIR}")


set(CMAKE_BUILD_TYPE Release)

set(DEMO_PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(DEMO_OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/lib/tof_dev_sdk_demo)

set(DEMO_INCLUDE_DIR ${DEMO_PROJECT_DIR}/include)

if (CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    # 处理 ARM 架构的情况
    set(DEMO_LIBS_DIR ${DEMO_PROJECT_DIR}/libs/aarch64)
elseif (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    # 处理 x86_64 架构的情况
    set(DEMO_LIBS_DIR ${DEMO_PROJECT_DIR}/libs/x86_64)
else ()
    message(FATAL_ERROR "Unsupported processor architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    return()
endif ()

set(DEMO_LIBS_NAME tof_dev_sdk pthread)
set(DEMO_PARAM_DIR ${DEMO_PROJECT_DIR}/parameter)

add_compile_options(-w)# Ignore all warnings
## Compile as C++11, supported in ROS Melodic and newer
add_compile_options(-std=c++14)
add_definitions(-DLINUX)


# for  -----------------------publisher_demo-----------------------------------
include_directories(${DEMO_INCLUDE_DIR}
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        )

#message(STATUS " DEMO_LIBS_DIR=========== ${DEMO_LIBS_DIR}")

link_directories(${DEMO_LIBS_DIR})

add_executable(publisher_demo demo/tof_dev_sdk_demo-ros-publisher.cpp)

target_link_libraries(publisher_demo ${DEMO_LIBS_NAME}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

#add_dependencies(talker XXXX_generate_messages_cpp)


#install(DIRECTORY "${DEMO_LIBS_DIR}/" DESTINATION "${DEMO_OUTPUT_DIR}" FILES_MATCHING PATTERN "*" PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ)
#install(DIRECTORY "${DEMO_PARAM_DIR}" DESTINATION "${DEMO_OUTPUT_DIR}")


## -----------------------------------------
## -----------------------------------------
## -----------------------------------------
## -----------------------------------------
## -----------------------------------------

add_library(sunny_tof_camera src/sunny_tof_camera.cpp)
add_dependencies(sunny_tof_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sunny_tof_camera ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${DEMO_LIBS_NAME})

add_library(nodelet_sunny_tof_camera src/nodelet_sunny_tof_camera.cpp)
add_dependencies(nodelet_sunny_tof_camera sunny_tof_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nodelet_sunny_tof_camera sunny_tof_camera ${catkin_LIBRARIES})

add_executable(sunny_tof_camera_node src/sunny_tof_camera_node.cpp)
target_link_libraries(sunny_tof_camera_node sunny_tof_camera ${catkin_LIBRARIES})

install(DIRECTORY
        launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(FILES sunny_tof_camera.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(TARGETS sunny_tof_camera nodelet_sunny_tof_camera sunny_tof_camera_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
