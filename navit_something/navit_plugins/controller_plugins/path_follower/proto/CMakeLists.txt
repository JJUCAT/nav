cmake_minimum_required(VERSION 3.0.2)
project(path_follower_proto)

function(PROTOBUF_GENERATE_PY SRCS)
  set(_protobuf_include_path -I ${CMAKE_CURRENT_SOURCE_DIR})

  set(${SRCS})
  foreach(FIL ${ARGN})
    get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
    get_filename_component(FIL_WE ${FIL} NAME_WE)
    if(NOT PROTOBUF_GENERATE_CPP_APPEND_PATH)
      get_filename_component(FIL_DIR ${FIL} DIRECTORY)
      if(FIL_DIR)
        set(FIL_WE "${FIL_DIR}/${FIL_WE}")
      endif()
    endif()

    #list(APPEND ${SRCS} "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}_pb2.py")
    #exec_program(/usr/bin/protoc ARGS --python_out ${CMAKE_CURRENT_BINARY_DIR} ${_protobuf_include_path} ${ABS_FIL})
  endforeach()

  set_source_files_properties(${${SRCS}} PROPERTIES GENERATED TRUE)
  set(${SRCS} ${${SRCS}} PARENT_SCOPE)
endfunction()

file(GLOB ProtoFiles *.proto)

PROTOBUF_GENERATE_CPP(ProtoSources ProtoHeaders ${ProtoFiles})

add_library(path_follower_proto SHARED ${ProtoSources})

target_link_libraries(path_follower_proto ${PROTOBUF_LIBRARIES})

#install(TARGETS common_proto
#        ARCHIVE DESTINATION lib
#        LIBRARY DESTINATION lib
#        RUNTIME DESTINATION bin
#)

# install(FILES ${ProtoPys} DESTINATION proto)
# install(FILES ${ProtoHeaders} ${ProtoSources} DESTINATION proto)

install(TARGETS path_follower_proto
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
