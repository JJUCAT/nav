cmake_minimum_required(VERSION 3.0.2)
project(navit_costmap)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            dynamic_reconfigure
            geometry_msgs
            laser_geometry
            map_msgs
            message_filters
            message_generation
            nav_msgs
            pluginlib
            roscpp
            sensor_msgs
            std_msgs
            tf2
            tf2_geometry_msgs
            tf2_ros
            tf2_sensor_msgs
            visualization_msgs
            voxel_grid
        )

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenCV REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

# messages
add_message_files(
    DIRECTORY msg
    FILES
    VoxelGrid.msg
)

generate_messages(
    DEPENDENCIES
        std_msgs
        geometry_msgs
        map_msgs
)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/NavitCostmap.cfg
    cfg/ObstaclePlugin.cfg
    cfg/GenericPlugin.cfg
    cfg/InflationPlugin.cfg
    cfg/VoxelPlugin.cfg
)

catkin_package(
    INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIRS}
    LIBRARIES navit_costmap navit_layers
    CATKIN_DEPENDS
        dynamic_reconfigure
        geometry_msgs
        laser_geometry
        map_msgs
        message_filters
        message_runtime
        nav_msgs
        pluginlib
        roscpp
        sensor_msgs
        std_msgs
        tf2_ros
        visualization_msgs
        voxel_grid
    DEPENDS
        EIGEN3
        Boost
)

add_library(navit_costmap
  src/array_parser.cpp
  src/costmap_2d.cpp
  src/observation_buffer.cpp
  src/layer.cpp
  src/layered_costmap.cpp
  src/costmap_2d_ros.cpp
  src/costmap_2d_publisher.cpp
  src/costmap_math.cpp
  src/footprint.cpp
  src/costmap_layer.cpp
  src/cleaned_map_ros.cpp
)
add_dependencies(navit_costmap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navit_costmap ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_library(navit_costmap_collision_checker src/collision_checker.cpp )
add_dependencies(navit_costmap_collision_checker navit_costmap)
target_link_libraries(navit_costmap_collision_checker navit_costmap)

add_library(navit_layers
  plugins/inflation_layer.cpp
  plugins/obstacle_layer.cpp
  plugins/static_layer.cpp
  plugins/voxel_layer.cpp
  src/observation_buffer.cpp
)
add_dependencies(navit_layers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navit_layers
  navit_costmap
)

add_executable(navit_costmap_markers src/costmap_2d_markers.cpp)
add_dependencies(navit_costmap_markers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navit_costmap_markers
    navit_costmap
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(navit_costmap_cloud src/costmap_2d_cloud.cpp)
add_dependencies(navit_costmap_cloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navit_costmap_cloud
    navit_costmap 
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(navit_costmap_node src/costmap_2d_node.cpp)
add_dependencies(navit_costmap_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navit_costmap_node
    navit_costmap
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

## Configure Tests
if(CATKIN_ENABLE_TESTING)
  # Find package test dependencies
  find_package(rostest REQUIRED)

  # Add the test folder to the include directories
  include_directories(test)

  # Create targets for test executables
  add_executable(costmap_tester EXCLUDE_FROM_ALL test/costmap_tester.cpp)
  add_dependencies(tests costmap_tester)
  target_link_libraries(costmap_tester navit_costmap ${GTEST_LIBRARIES})

  add_executable(footprint_tests EXCLUDE_FROM_ALL test/footprint_tests.cpp)
  add_dependencies(tests footprint_tests)
  target_link_libraries(footprint_tests navit_costmap ${GTEST_LIBRARIES})

  add_executable(obstacle_tests EXCLUDE_FROM_ALL test/obstacle_tests.cpp)
  add_dependencies(tests obstacle_tests)
  target_link_libraries(obstacle_tests navit_costmap navit_layers ${GTEST_LIBRARIES})

  add_executable(static_tests EXCLUDE_FROM_ALL test/static_tests.cpp)
  add_dependencies(tests static_tests)
  target_link_libraries(static_tests navit_costmap navit_layers ${GTEST_LIBRARIES})

  add_executable(collision_checker_tests EXCLUDE_FROM_ALL test/collision_checker_tests.cpp)
  add_dependencies(tests collision_checker_tests)
  target_link_libraries(collision_checker_tests navit_costmap navit_layers navit_costmap_collision_checker ${GTEST_LIBRARIES})

  # add_executable(inflation_tests EXCLUDE_FROM_ALL test/inflation_tests.cpp)
  # add_dependencies(tests inflation_tests)
  # target_link_libraries(inflation_tests navit_costmap layers ${GTEST_LIBRARIES})

  # catkin_download_test_data(${PROJECT_NAME}_simple_driving_test_indexed.bag
  #   http://download.ros.org/data/costmap_2d/simple_driving_test_indexed.bag
  #   DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
  #   MD5 61168cff9425b11e093ea3a627c81c8d)
  # catkin_download_test_data(${PROJECT_NAME}_willow-full-0.025.pgm
  #   http://download.ros.org/data/costmap_2d/willow-full-0.025.pgm
  #   DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
  #   MD5 e66b17ee374f2d7657972efcb3e2e4f7)

  add_rostest(test/footprint_tests.launch)
  # add_rostest(test/inflation_tests.launch)
  add_rostest(test/obstacle_tests.launch)
  #add_rostest(test/simple_driving_test.xml)
  add_rostest(test/static_tests.launch)
  add_rostest(test/collision_checker_tests.launch)

  catkin_add_gtest(array_parser_test test/array_parser_test.cpp)
  target_link_libraries(array_parser_test navit_costmap)

  catkin_add_gtest(coordinates_test test/coordinates_test.cpp)
  target_link_libraries(coordinates_test navit_costmap)

endif()
# use this to download rosbag data
# file(DOWNLOAD
# http://192.168.9.53:8000/bags/download?bagId=3
# ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test/test.bag
# SHOW_PROGRESS)

install( TARGETS
    navit_costmap_markers
    navit_costmap_cloud
    navit_costmap_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    navit_costmap
    navit_layers
    navit_costmap_collision_checker
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES test/TenByTen.yaml test/TenByTen.pgm
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
