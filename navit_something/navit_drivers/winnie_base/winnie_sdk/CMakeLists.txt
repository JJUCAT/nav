

#include_directories(include)
add_library(winnie_sdk
        SHARED
        src/execution.cpp
        src/handle.cpp
        src/protocol.cpp
        src/serial_device.cpp
        )
target_link_libraries(winnie_sdk
        PUBLIC
        Threads::Threads
        )
target_include_directories(winnie_sdk
        PUBLIC
        include)