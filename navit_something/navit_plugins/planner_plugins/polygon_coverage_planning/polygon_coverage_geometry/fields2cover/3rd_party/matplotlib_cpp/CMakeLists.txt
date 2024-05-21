cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(matplotlib_cpp LANGUAGES CXX)

include(GNUInstallDirs)
set(PACKAGE_NAME matplotlib_cpp)
set(USING_PYTHON3 True)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH
  "Installation directory for header files")

foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

# Library target
add_library(matplotlib_cpp INTERFACE)
target_include_directories(matplotlib_cpp
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(matplotlib_cpp INTERFACE cxx_std_17)

# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
if(USING_PYTHON3)
  find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
  target_link_libraries(matplotlib_cpp INTERFACE
    Python3::Python
    Python3::Module
  )
  find_package(Python3 COMPONENTS NumPy)
  if(Python3_NumPy_FOUND)
    target_link_libraries(matplotlib_cpp INTERFACE
      Python3::NumPy
    )
  else()
    target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
  endif()
else()
  find_package(Python2 COMPONENTS Interpreter Development REQUIRED)
  target_link_libraries(matplotlib_cpp INTERFACE
    Python2::Python
    Python2::Module
  )
  find_package(Python2 COMPONENTS NumPy)
  if(Python2_NumPy_FOUND)
    target_link_libraries(matplotlib_cpp INTERFACE
      Python2::NumPy
    )
  else()
    target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
  endif()
endif()



install(TARGETS matplotlib_cpp
  EXPORT matplotlib_cpp-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/matplotlib_cpp"
)

# Install targets file
install(EXPORT matplotlib_cpp-targets
  FILE ${PACKAGE_NAME}Targets.cmake
  DESTINATION ${INSTALL_CMAKE_DIR}/cmake/matplotlib_cpp
)

# Install matplotlib_cppConfig.cmake
include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PACKAGE_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake
  INSTALL_DESTINATION ${INSTALL_CMAKE_DIR}/cmake/matplotlib_cpp
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake
  DESTINATION ${INSTALL_CMAKE_DIR}/cmake/matplotlib_cpp
)

export(TARGETS matplotlib_cpp
  FILE ${PACKAGE_NAME}Config.cmake)

# Install headers
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/matplotlibcpp.h"
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)



set(CMAKE_EXPORT_PACKAGE_REGISTRY ON)
export(PACKAGE matplotlib_cpp)
