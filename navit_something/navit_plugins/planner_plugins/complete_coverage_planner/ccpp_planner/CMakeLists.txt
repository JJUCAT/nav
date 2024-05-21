aux_source_directory(src/. SRC_LIST)

add_library(${CCPP_LIB_NAME} SHARED ${SRC_LIST})
target_include_directories(${CCPP_LIB_NAME} PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${CCPP_LIB_NAME} ${OpenCV_LIBRARIES})
add_executable(${CCPP_LIB_NAME}_test test/ccpp_test.cpp)
target_link_libraries(${CCPP_LIB_NAME}_test ${CCPP_LIB_NAME})

return()

cmake_minimum_required(VERSION 3.0.2)
project(ccpp_planner)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

find_package(OpenCV REQUIRED)

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
)

aux_source_directory(src/. SRC_LIST)

if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
add_library(ccpp_aarch64 SHARED ${SRC_LIST})
target_link_libraries(ccpp_aarch64 ${OpenCV_LIBRARIES})
add_executable(ccpp_test_aarch64 test/ccpp_test.cpp)
target_link_libraries(ccpp_test_aarch64 ccpp_aarch64)
set_target_properties(ccpp_aarch64 ccpp_test_aarch64 
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../lib"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../bin"
)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
add_library(ccpp_x86 SHARED ${SRC_LIST})
target_link_libraries(ccpp_x86 ${OpenCV_LIBRARIES})
add_executable(ccpp_test_x86 test/ccpp_test.cpp)
target_link_libraries(ccpp_test_x86 ccpp_x86)
set_target_properties(ccpp_x86 ccpp_test_x86
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../lib"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../bin"
)
endif()
