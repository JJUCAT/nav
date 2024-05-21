cmake_minimum_required(VERSION 3.2)
project(fcpp_algorithm)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)
# if(CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE)
  set(catkin_FOUND 1)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES fcpp_algorithm
    CATKIN_DEPENDS
    DEPENDS 
  )
# endif()
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CGAL QUIET COMPONENTS Core)
## add package config
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CGAL_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE srcs "src/*.cpp")
file(GLOB_RECURSE hdrs "include/*.h")

message("find source files: ${srcs}")
message("find headers: ${hdrs}")

set(FCPP_ALGORITHM_DEPENDENCIES ${OpenCV_LIBS} CGAL::CGAL CGAL::CGAL_Core glog)

add_library(fcpp_algorithm SHARED ${srcs} ${hdrs})
add_dependencies(fcpp_algorithm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fcpp_algorithm ${FCPP_ALGORITHM_DEPENDENCIES})

## add the test 
add_executable(fcpp_singlebow_test test/test_singlebow.cpp)
target_link_libraries(fcpp_singlebow_test fcpp_algorithm)

add_executable(fcpp_doublebow_test test/test_doublebow.cpp)
target_link_libraries(fcpp_doublebow_test fcpp_algorithm)

set(PACKAGE_CONFIG_VERSION_FILE "${CMAKE_BINARY_DIR}/fcpp_algorithmConfig-version.cmake")
set(PACKAGE_CONFIG_FILE "${CMAKE_BINARY_DIR}/fcpp_algorithmConfig.cmake")

if(catkin_FOUND)
   set(FCPP_ALGOTIEHM_LIB_DESTINATION   ${CATKIN_PACKAGE_LIB_DESTINATION} )
   set(FCPP_ALGOTIEHM_BIN_DESTINATION   ${CATKIN_PACKAGE_BIN_DESTINATION} )
   set(FCPP_ALGOTIEHM_INC_DESTINATION   ${CATKIN_PACKAGE_INCLUDE_DESTINATION} )
else()
   set(FCPP_ALGOTIEHM_LIB_DESTINATION   ${CMAKE_INSTALL_LIBDIR} )
   set(FCPP_ALGOTIEHM_BIN_DESTINATION   ${CMAKE_INSTALL_BINDIR} )
   set(FCPP_ALGOTIEHM_INC_DESTINATION   ${CMAKE_INSTALL_INCLUDEDIR} )
endif()

if(catkin_FOUND)

else()
set(FCPP_ALGHRITHM_INSTALL_DIR "${CMAKE_INSTALL_DATADIR}/fcpp_algorithm")
set(FCPP_ALGHRITHM_INSTALL_CMAKE_DIR "${FCPP_ALGHRITHM_INSTALL_DIR}/cmake")

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/fcpp_algorithmConfig.cmake.in"
   ${PACKAGE_CONFIG_FILE}
   INSTALL_DESTINATION ${FCPP_ALGHRITHM_INSTALL_CMAKE_DIR}
)
write_basic_package_version_file(
  ${PACKAGE_CONFIG_VERSION_FILE}
  COMPATIBILITY ExactVersion
)
install(
  FILES
    ${PACKAGE_CONFIG_VERSION_FILE}
    ${PACKAGE_CONFIG_FILE}
  DESTINATION ${FCPP_ALGHRITHM_INSTALL_CMAKE_DIR}
)
install(
  FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/package.xml"
  DESTINATION ${FCPP_ALGHRITHM_INSTALL_DIR}
)
endif()

install(
  TARGETS fcpp_algorithm 
  RUNTIME DESTINATION ${FCPP_ALGOTIEHM_BIN_DESTINATION}
  LIBRARY DESTINATION ${FCPP_ALGOTIEHM_LIB_DESTINATION}
  ARCHIVE DESTINATION ${FCPP_ALGOTIEHM_LIB_DESTINATION}
)

install(
  DIRECTORY include/
  DESTINATION ${FCPP_ALGOTIEHM_INC_DESTINATION}
)


