cmake_minimum_required(VERSION 3.15.2)
project(berxel_camera)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        # camera_info_manager
        image_transport
        roscpp
        rospy
        std_msgs
        tf
        nodelet
        )

catkin_package(

)


if (CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    # 处理 ARM 架构的情况
    set(SDK_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sdk/libs/aarch64)
elseif (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    # 处理 x86_64 架构的情况
    set(SDK_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sdk/libs/x86_64)
else ()
    message(FATAL_ERROR "Unsupported processor architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    return()
endif ()


include_directories(
        ${catkin_INCLUDE_DIRS}
        include
        sdk/include
)

#if(DEFINED ENV{BERXEL_SDK_LIBRARY})
#    set(BERXEL_SDK_INCLUDE $ENV{BERXEL_SDK_LIBRARY}/Include)
#    set(BERXEL_SDK_LIB $ENV{BERXEL_SDK_LIBRARY}/libs)
#    include_directories(${BERXEL_SDK_INCLUDE})
#    LINK_DIRECTORIES(${BERXEL_SDK_LIB})
#else()
#    message(FATAL_ERROR "NOT FOUND BERXEL_SDK_INCLUDE, PLEASE SET BERXEL_SDK_INCLUDE")
#endif()

add_library(nodelet_${PROJECT_NAME} src/nodelet_berxel_camera.cpp src/BerxelCommonFunc.cpp src/BerxelHawkCamera.cpp)
target_link_directories(nodelet_${PROJECT_NAME} PRIVATE ${SDK_LIBS_DIR})
target_link_libraries(
        nodelet_${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        BerxelHawk
)

add_executable(${PROJECT_NAME} src/BerxelRebotCamera.cpp src/BerxelCommonFunc.cpp src/BerxelHawkCamera.cpp)
target_link_directories(${PROJECT_NAME} PRIVATE ${SDK_LIBS_DIR})
target_link_libraries(
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        BerxelHawk
)

# 在构建后复制SDK库文件到devel目录
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${SDK_LIBS_DIR} ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY ${SDK_LIBS_DIR}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
