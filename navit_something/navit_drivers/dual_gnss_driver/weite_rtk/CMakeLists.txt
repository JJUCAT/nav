cmake_minimum_required(VERSION 2.8.3)
project(weite_rtk)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++11)
add_definitions(-std=c++11)


find_package(
  catkin REQUIRED
  COMPONENTS navit_msgs
             diagnostic_msgs
             diagnostic_updater
             roscpp
             rospy
             std_msgs
             geometry_msgs
             sensor_msgs
             nav_msgs
             tf
             eigen_conversions
             nodelet
			 serial
			# glog
			 )
# message("${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}:" 
# ${CMAKE_CURRENT_SOURCE_DIR}/include/integrated_navigation/)
catkin_package(CATKIN_DEPENDS navit_msgs)

set(ALL_TARGET_LIBRARIES "")
include(cmake/Boost.cmake)
#include(cmake/glog.cmake)
#include(cmake/sophus.cmake)

include_directories(include ${catkin_INCLUDE_DIRS}
${CMAKE_CURRENT_SOURCE_DIR}/include/integrated_navigation/
${CMAKE_CURRENT_SOURCE_DIR}/include/integrated_navigation/glog/
)

#file(GLOB_RECURSE ALL_SRCS "*.cpp")
#file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp")
#list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})

# ########################## sins_gnss_filtering_nodelet
# ##############################################################################
# set(SINS_GNSS_FILTER_NODELET_SRC
#     src/apps/filter/sins_gnss_filtering_nodelet.cpp
#     src/filtering/sins_rtk_filtering_flow.cpp
#     src/filtering/sins_rtk_filtering.cpp
# 	src/models/error_state_kalman_filter/filter_initializer.cpp
#     src/models/error_state_kalman_filter/filter_predictor.cpp
#     src/models/error_state_kalman_filter/gnss_corrector/gnss_corrector.cpp
# 	src/models/error_state_kalman_filter/wheel_corrector/wheel_corrector.cpp
# 	src/models/error_state_kalman_filter/dzupt_corrector/dzupt_corrector.cpp
# 	src/models/error_state_kalman_filter/filter_corrector_common.cpp
# 	src/models/error_state_kalman_filter/filter_checker.cpp
# 	src/library/GeographicLib/Geocentric.cpp
#     src/library/GeographicLib/LocalCartesian.cpp
#     src/library/GeographicLib/Math.cpp
#     src/subscriber/imu_subscriber.cpp
#     src/subscriber/gnss_subscriber.cpp
# 	src/subscriber/wheel_subscriber.cpp
# 	src/publisher/ins_info_publisher.cpp
# 	src/publisher/nav_sat_fix_publisher.cpp
# 	src/publisher/navsatfix_health_publisher.cpp
#     src/publisher/odometry_publisher.cpp
# 	src/publisher/pose_stamped_publisher.cpp
#     src/publisher/tf_broadcaster.cpp
#     src/publisher/imu_publisher.cpp
# 	src/publisher/debug_publisher.cpp
#     src/sensor_data/velocity_data.cpp
# 	src/utils/utils.cpp
# 	src/models/complementary_filter/complementary_filter.cpp
# 	)

# add_library(${PROJECT_NAME}_filter_nodelet ${SINS_GNSS_FILTER_NODELET_SRC})
# target_link_libraries(${PROJECT_NAME}_filter_nodelet ${catkin_LIBRARIES}
#                       ${ALL_TARGET_LIBRARIES})
# add_dependencies(${PROJECT_NAME}_filter_nodelet
#                  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# ##############################################################################

########################## multi_sensor_driver_nodelet
##############################################################################
set(MULTI_SENSOR_DRIVER_NODELET_SRC
	src/apps/driver/multi_sensor/multi_sensor_driver_nodelet.cpp
	src/driver/io_common/serial_common.cpp
	src/driver/io_common/tcp_common.cpp
	src/driver/multi_sensor_driver.cpp
	src/driver/imu/ky102n_imu_driver.cpp
	src/driver/gnss/novatel_gps_driver.cpp
	src/driver/gnss/NovatelGps.cpp
	src/driver/radio/radio_message_driver.cpp
	src/library/GnssParserLib/novatel_message_extractor.cpp
	src/library/GnssParserLib/parsers/bestpos.cpp
	src/library/GnssParserLib/parsers/bestvel.cpp
	src/library/GnssParserLib/parsers/bestxyz.cpp
	src/library/GnssParserLib/parsers/dual_antenna_heading.cpp
	src/library/GnssParserLib/parsers/gpchc.cpp
	src/library/GnssParserLib/parsers/gpgga.cpp
	#src/library/GnssParserLib/parsers/gpgsa.cpp
	src/library/GnssParserLib/parsers/gprmc.cpp
	src/library/GnssParserLib/parsers/header.cpp
	src/library/GnssParserLib/parsers/parsing_utils.cpp
	src/library/GnssParserLib/parsers/refstation.cpp
	src/library/GnssParserLib/parsers/time.cpp
	# src/models/complementary_filter/complementary_filter.cpp
	src/library/Serial/serial_utils.cpp
	src/library/Serial/string_utils.cpp
	src/publisher/gpchc_publisher.cpp
	)

add_library(${PROJECT_NAME}_multi_sensor_nodelet ${MULTI_SENSOR_DRIVER_NODELET_SRC} )
target_link_libraries(${PROJECT_NAME}_multi_sensor_nodelet ${catkin_LIBRARIES}
		${Boost_LIBRARIES} ${ALL_TARGET_LIBRARIES})
add_dependencies(${PROJECT_NAME}_multi_sensor_nodelet
		${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# ##############################################################################

# ########################## system_status_control_nodelet
# ##############################################################################
# set(SYSTEM_CONTROL_NODELET_SRC
#     src/apps/control/system_status_control_nodelet.cpp
# 	src/control/system_status_control.cpp
# 	src/publisher/system_control_publisher.cpp)

# add_library(${PROJECT_NAME}_control_nodelet ${SYSTEM_CONTROL_NODELET_SRC})
# target_link_libraries(${PROJECT_NAME}_control_nodelet ${catkin_LIBRARIES}
#                       ${Boost_LIBRARIES} ${ALL_TARGET_LIBRARIES})
# add_dependencies(${PROJECT_NAME}_control_nodelet
#                  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# ##############################################################################

# ########################## ins_driver_nodelet
# ##############################################################################
set(INS_DRIVER_NODELET_SRC
    src/apps/driver/ins/ins_driver_nodelet.cpp
	src/driver/ins_sensor_driver.cpp
	src/driver/io_common/serial_common.cpp
	src/driver/io_common/tcp_common.cpp
	src/driver/gnss/novatel_gps_driver.cpp
	src/driver/gnss/NovatelGps.cpp
	src/library/GnssParserLib/novatel_message_extractor.cpp
	src/library/GnssParserLib/parsers/bestpos.cpp
	src/library/GnssParserLib/parsers/bestvel.cpp
	src/library/GnssParserLib/parsers/bestxyz.cpp
	src/library/GnssParserLib/parsers/dual_antenna_heading.cpp
	src/library/GnssParserLib/parsers/gpchc.cpp
	src/library/GnssParserLib/parsers/gpgga.cpp
	#src/library/GnssParserLib/parsers/gpgsa.cpp
	src/library/GnssParserLib/parsers/gprmc.cpp
	src/library/GnssParserLib/parsers/header.cpp
	src/library/GnssParserLib/parsers/parsing_utils.cpp
	src/library/GnssParserLib/parsers/refstation.cpp
	src/library/GnssParserLib/parsers/time.cpp
	src/library/Serial/serial_utils.cpp
	src/library/Serial/string_utils.cpp
	src/publisher/gpchc_publisher.cpp
	)

add_library(${PROJECT_NAME}_ins_nodelet ${INS_DRIVER_NODELET_SRC} )
target_link_libraries(${PROJECT_NAME}_ins_nodelet ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES} ${ALL_TARGET_LIBRARIES})
add_dependencies(${PROJECT_NAME}_ins_nodelet
                 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# ##############################################################################

# ########################## pose_debug_nodelet
# ##############################################################################
# set(POSE_DEBUG_NODELET_SRC
# 	src/apps/debug/pose_debug_nodelet.cpp
# 	src/debug/pose_debug.cpp
# 	src/library/GeographicLib/Geocentric.cpp
# 	src/library/GeographicLib/LocalCartesian.cpp
# 	src/library/GeographicLib/Math.cpp)

# add_library(${PROJECT_NAME}_debug_nodelet ${POSE_DEBUG_NODELET_SRC})
# target_link_libraries(${PROJECT_NAME}_debug_nodelet ${catkin_LIBRARIES}
# 		${Boost_LIBRARIES} ${ALL_TARGET_LIBRARIES})
# add_dependencies(${PROJECT_NAME}_debug_nodelet
# 		${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# ##############################################################################

# ################################ nodelet install
# ##############################################################################
install(TARGETS
		#${PROJECT_NAME}_filter_nodelet
		${PROJECT_NAME}_multi_sensor_nodelet
		#${PROJECT_NAME}_control_nodelet
		${PROJECT_NAME}_ins_nodelet
		#${PROJECT_NAME}_debug_nodelet
  		RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES nodelet.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
# install(DIRECTORY config/filtering/
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config/filtering)
install(DIRECTORY config/sensor/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config/sensor)
install(DIRECTORY rviz/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)
# ##############################################################################