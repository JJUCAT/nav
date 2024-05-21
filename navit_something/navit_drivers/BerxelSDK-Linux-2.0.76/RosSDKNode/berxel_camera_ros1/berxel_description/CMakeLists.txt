cmake_minimum_required(VERSION 2.8.12)
project(berxel_description)

find_package(catkin REQUIRED COMPONENTS
    )

catkin_package(
    )

install(DIRECTORY launch meshes rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
