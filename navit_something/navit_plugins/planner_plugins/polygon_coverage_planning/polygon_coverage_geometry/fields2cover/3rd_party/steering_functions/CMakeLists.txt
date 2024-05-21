cmake_minimum_required(VERSION 3.11)
project(steering_functions)
set(PROJECT_MAJOR_VERSION "1")
set(PROJECT_MINOR_VERSION "0")
set(PROJECT_PATCH_VERSION "0")
set(PROJECT_VERSION_STRING "${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION}.${PROJECT_PATCH_VERSION}")
set(PROJECT_VERSION ${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION}.${PROJECT_PATCH_VERSION})

find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

## Get all source files
file(GLOB SRC_HC_CC_SS src/hc_cc_state_space/*.cpp)
set(SRC_FILES
    src/dubins_state_space/dubins_state_space.cpp
    src/reeds_shepp_state_space/reeds_shepp_state_space.cpp
    src/utilities/utilities.cpp
    ${SRC_HC_CC_SS})

## Declare a C++ library
add_library(steering_functions ${SRC_FILES})
target_compile_features(steering_functions PUBLIC cxx_std_11)


target_include_directories(steering_functions PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(steering_functions)

set_target_properties(steering_functions
                      PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/steering_functions"
)

export(PACKAGE steering_functions)
