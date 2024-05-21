
aux_source_directory(src/parsers SOURCES)

add_library(gnss_parser2 ${SOURCES})
target_include_directories(gnss_parser2 PUBLIC include)
add_dependencies(gnss_parser2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gnss_parser2 ${catkin_LIBRARIES})