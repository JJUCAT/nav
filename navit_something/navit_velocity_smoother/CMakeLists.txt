cmake_minimum_required(VERSION 3.0.2)
project(navit_velocity_smoother)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
    dynamic_reconfigure 
    geometry_msgs 
    nav_msgs 
    roscpp 
    nodelet
    ackermann_msgs
	message_generation
    )

generate_dynamic_reconfigure_options(cfg/params.cfg)

catkin_package(
  CATKIN_DEPENDS 
  dynamic_reconfigure 
  geometry_msgs 
  nav_msgs 
  roscpp 
  ackermann_msgs
)

### BUILD ###
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} 
    src/velocity_smoother.cpp 
    src/twist_velocity_smoother.cpp 
    src/ackermann_velocity_smoother.cpp 
    src/velocity_smoother_node.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${BOOST_LIBRARIES})

add_library(${PROJECT_NAME}_nodelet
    src/velocity_smoother.cpp
    src/twist_velocity_smoother.cpp
    src/ackermann_velocity_smoother.cpp
    )
add_dependencies(${PROJECT_NAME}_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_nodelet 
    ${catkin_LIBRARIES}
    )

### INSTALL ###
catkin_install_python(PROGRAMS
   scripts/test_ackermann_vel_smoother.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS 
    ${PROJECT_NAME} 
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS 
    ${PROJECT_NAME}_nodelet
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
    velocity_smoother_nodelet.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY 
    cfg 
    config
    launch 
    scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
