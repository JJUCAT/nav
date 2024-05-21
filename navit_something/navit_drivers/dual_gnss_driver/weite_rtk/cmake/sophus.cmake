find_package (Sophus REQUIRED)

include_directories(${Sophus_INCLUDE_DIRS})
#include_directories("/usr/local/include/eigen3")
list(APPEND ALL_TARGET_LIBRARIES ${Sophus_LIBRARIES})