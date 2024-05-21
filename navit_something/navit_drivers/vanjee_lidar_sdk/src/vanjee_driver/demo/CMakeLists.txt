cmake_minimum_required(VERSION 3.5)

project(vanjee_driver_demos)

message(=============================================================)
message("-- Ready to compile demos")
message(=============================================================)

if (${ENABLE_PCL_POINTCLOUD})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

endif (${ENABLE_PCL_POINTCLOUD})

find_package(vanjee_driver REQUIRED)
include_directories(${VANJEE_DRIVER_INCLUDE_DIRS})
add_definitions("-DDISABLE_PCAP_PARSE")

add_executable(demo_online
              demo_online.cpp)
target_link_libraries(demo_online
                    ${EXTERNAL_LIBS} ${VANJEE_DRIVER_LIBRARIES})

add_executable(demo_online_multi_lidars
              demo_online_multi_lidars.cpp)

target_link_libraries(demo_online_multi_lidars
                    ${EXTERNAL_LIBS} ${VANJEE_DRIVER_LIBRARIES})

if(NOT ${DISABLE_PCAP_PARSE})

add_executable(demo_pcap
               demo_pcap.cpp)

target_link_libraries(demo_pcap
                     ${EXTERNAL_LIBS} ${VANJEE_DRIVER_LIBRARIES})
                     
endif(NOT ${DISABLE_PCAP_PARSE})


        
