cmake_minimum_required(VERSION 2.8.3)
project(vanjee_driver)

add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)#RelWithDebInfo
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(${PROJECT_NAME}_CATKIN_DEPS
    angles
    pcl_ros
    roscpp
    sensor_msgs
    tf
    dynamic_reconfigure
    vanjee_msgs
    nodelet
)

set(libpcap_LIBRARIES -lpcap)

find_package(catkin REQUIRED COMPONENTS
	${${PROJECT_NAME}_CATKIN_DEPS}
  pcl_conversions
  rospy
  std_msgs
  genmsg
  cv_bridge
  message_generation
  nodelet
  roscpp
  # common
	# common_proto
	# common_module_status
)

find_package(yaml-cpp REQUIRED)
find_package(Boost COMPONENTS signals)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(PkgConfig REQUIRED)


include_directories(
		include
		${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}
	${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
# Generate dynamic_reconfigure server
generate_dynamic_reconfigure_options(cfg/VanjeeNode.cfg)
catkin_package(
	LIBRARIES vanjee_driver
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
    CATKIN_DEPENDS message_runtime std_msgs
    )


add_library(vanjee_input src/driver720.cpp src/driver721.cpp)
target_link_libraries(vanjee_input
		${catkin_LIBRARIES}
		${libpcap_LIBRARIES}
    ${YAML_CPP_LIBRARIES})

add_library(vanjee_driver src/vanjee_driver.cpp)
target_link_libraries(vanjee_driver
		vanjee_input
		${catkin_LIBRARIES})




# build the nodelet version
add_library(vanjee_driver_nodelet src/vanjee_nodelet.cc src/vanjee_driver.cpp)
target_link_libraries(vanjee_driver_nodelet
		vanjee_input
		${catkin_LIBRARIES}
		)

add_executable(vanjee_driver_node src/vanjee_driver_node.cpp)

if(catkin_EXPORTED_TARGETS)
	add_dependencies(vanjee_input ${catkin_EXPORTED_TARGETS})
endif()

target_link_libraries(vanjee_driver_node
		vanjee_driver
		vanjee_input
		${catkin_LIBRARIES}
		${libpcap_LIBRARIES}
		)

install(TARGETS vanjee_driver_node DESTINATION ./lib/vanjee_driver)
install(DIRECTORY cfg launch DESTINATION ./share/vanjee_driver)
install(FILES nodelet_vanjee.xml DESTINATION ./share/vanjee_driver)
install(TARGETS vanjee_driver_nodelet vanjee_driver vanjee_input DESTINATION ./lib)
