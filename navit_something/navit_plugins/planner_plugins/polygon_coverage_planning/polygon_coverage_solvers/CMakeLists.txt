cmake_minimum_required(VERSION 3.16.3)
project(polygon_coverage_solvers)

find_package(catkin REQUIRED COMPONENTS rosconsole roslib)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

catkin_package(
        INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
        LIBRARIES ${PROJECT_NAME}
)
message("CMAKE_CURRENT_BINARY_DIR is ${CMAKE_CURRENT_BINARY_DIR}")

include_directories(include ${catkin_INCLUDE_DIRS})
message("CMAKE_CURRENT_BINARY_DIR is ${CMAKE_CURRENT_BINARY_DIR}")

# Add mono to invoke gk_ma.
find_package(PkgConfig)
pkg_check_modules(MONO mono-2 REQUIRED)
include_directories(${MONO_INCLUDE_DIRS})
# Download gk_ma
set(GKMA_INCLUDE_DIR ${CMAKE_BINARY_DIR}/gk_ma-prefix/src/gk_ma)
include(ExternalProject)
ExternalProject_Add(
  gk_ma
  URL ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/gk_ma
  DOWNLOAD_NAME gk_ma
  PATCH_COMMAND ""
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND
    COMMAND cp ${PROJECT_SOURCE_DIR}/patches/MakefileCpp ./MakefileCpp
    COMMAND cp ${PROJECT_SOURCE_DIR}/patches/MakefileCs ./MakefileCs
  BUILD_COMMAND
    COMMAND $(MAKE) -f MakefileCs BUILD_PATH="${CMAKE_LIBRARY_OUTPUT_DIRECTORY}"
    COMMAND $(MAKE) -f MakefileCpp BUILD_PATH="${CMAKE_LIBRARY_OUTPUT_DIRECTORY}"
  INSTALL_COMMAND
    COMMAND ${CMAKE_COMMAND} -E create_symlink /usr/lib/libmono-native.so ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/System.Native
)
# cmakelist当前目录

set(MY_CUSTOM_LIB_DESTINATION "share/polygon_coverage_solvers/devel/lib")

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/bin/GkMa.exe
         PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ WORLD_READ WORLD_EXECUTE
         DESTINATION ${MY_CUSTOM_LIB_DESTINATION})

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/bin/libnative_helper.so
         PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ WORLD_READ WORLD_EXECUTE
         DESTINATION ${MY_CUSTOM_LIB_DESTINATION})
#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}
  src/gk_ma.cc
  src/combinatorics.cc
  src/boolean_lattice.cc
)
target_link_libraries(${PROJECT_NAME} ${MONO_LIBRARIES} ${catkin_LIBRARIES})

#########
# TESTS #
#########
catkin_add_gtest(test_combinatorics test/combinatorics-test.cpp)
target_link_libraries(test_combinatorics ${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_add_gtest(test_gk_ma test/gk_ma-test.cpp)
target_link_libraries(test_gk_ma ${PROJECT_NAME} ${catkin_LIBRARIES})


##########
# EXPORT #
##########
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )
