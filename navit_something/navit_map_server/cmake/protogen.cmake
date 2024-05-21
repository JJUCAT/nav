find_package(Protobuf REQUIRED)

set(proto_dir ${PROJECT_SOURCE_DIR}/proto)
file(GLOB proto_files "${proto_dir}/*.proto")

# Set up destination directories
catkin_destinations()
set(proto_gen_dir ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})
set(proto_gen_cpp_dir ${proto_gen_dir})
file(MAKE_DIRECTORY ${proto_gen_dir})
file(MAKE_DIRECTORY ${proto_gen_cpp_dir})
set(protogen_include_dirs ${proto_gen_cpp_dir}/../ ${proto_gen_python_dir})

# Create lists of files to be generated
set(proto_gen_cpp_files "")
foreach (proto_file ${proto_files})
    get_filename_component(proto_name ${proto_file} NAME_WE)
    list(APPEND proto_gen_cpp_files
            ${proto_gen_cpp_dir}/${proto_name}.pb.h
            ${proto_gen_cpp_dir}/${proto_name}.pb.cc
            )
endforeach (proto_file ${proto_files})

# Run protoc and generate language-specific headers.
add_custom_command(
        OUTPUT ${proto_gen_cpp_files}
        COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} --proto_path=${proto_dir} --cpp_out=${proto_gen_cpp_dir} ${proto_files}
        DEPENDS ${PROTOBUF_PROTOC_EXECUTABLE} ${proto_files}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)