# Find gRPC
find_package(Protobuf REQUIRED)

# Generated gRPC files
get_filename_component(navigation_proto "./proto/navigation.proto" ABSOLUTE)
get_filename_component(navigation_proto_path "${navigation_proto}" PATH)
set(navigation_proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/navigation.pb.cc")
set(navigation_proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/navigation.pb.h")
set(navigation_grpc_srcs "${CMAKE_CURRENT_BINARY_DIR}/navigation.grpc.pb.cc")
set(navigation_grpc_hdrs "${CMAKE_CURRENT_BINARY_DIR}/navigation.grpc.pb.h")
add_custom_command(
        OUTPUT "${navigation_proto_srcs}" "${navigation_proto_hdrs}" "${navigation_grpc_srcs}" "${navigation_grpc_hdrs}"
        COMMAND protoc
        ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
        --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
        -I "${navigation_proto_path}"
        --plugin=protoc-gen-grpc=`which grpc_cpp_plugin`
        "${navigation_proto}"
        DEPENDS "${navigation_proto}"
)

# Executable
add_library(navit_bt_nodes_task_receiver
        src/task_receiver_action.cpp
        ${navigation_proto_srcs}
        ${navigation_grpc_srcs}
        )
target_include_directories(navit_bt_nodes_task_receiver PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
add_dependencies(navit_bt_nodes_task_receiver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navit_bt_nodes_task_receiver ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} grpc++)
target_compile_definitions(navit_bt_nodes_task_receiver PRIVATE BT_PLUGIN_EXPORT)
