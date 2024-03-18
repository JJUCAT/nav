add_executable(exampleEDT3D exampleEDT3D.cpp)
target_link_libraries(exampleEDT3D dynEDT3D)

add_executable(exampleEDTOctomap exampleEDTOctomap.cpp)
target_link_libraries(exampleEDTOctomap dynEDT3D ${OCTOMAP_LIBRARIES})
