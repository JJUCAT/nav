cmake_minimum_required(VERSION 3.0.2)
project(navit_map_server)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        geometry_msgs
        tf2
        navit_msgs
        )

find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(YAML-CPP REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})
find_package(OpenCV REQUIRED)

include(cmake/protogen.cmake)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        navit_map_server
        CATKIN_DEPENDS
        roscpp
        geometry_msgs
        tf2
)

include_directories(
        include
        ${protogen_include_dirs}
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_library(navit_map_server src/navit_map_server.cpp ${proto_gen_cpp_files})
add_dependencies(navit_map_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navit_map_server
        ${catkin_LIBRARIES}
        ${PROTOBUF_LIBRARIES}
        ${OpenCV_LIBS}
        ${YAML_CPP_LIBRARIES}
        ${Boost_LIBRARIES}
        )

add_executable(navit_map_server_node src/navit_map_server_node.cpp)
add_dependencies(navit_map_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navit_map_server_node
        navit_map_server
        ${catkin_LIBRARIES}
        )

# add_executable(navit_ogm_generator_node src/ogm_generator_node.cpp)
# add_dependencies(navit_ogm_generator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(navit_ogm_generator_node
#         navit_map_server
#         ${catkin_LIBRARIES}
#         ${OpenCV_LIBS}
#         )
catkin_install_python(PROGRAMS scripts/geojson_loader.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(TARGETS navit_map_server_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Install executables and/or libraries
install(TARGETS navit_map_server navit_map_server_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
        config
        launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
