cmake_minimum_required(VERSION 3.9 FATAL_ERROR)
project (libslic3r)


# only on newer GCCs: -ftemplate-backtrace-limit=0
add_compile_options(-ftemplate-backtrace-limit=0)
add_compile_options(-DNO_PERL -DM_PI=3.14159265358979323846 -DHAS_BOOL -DNOGDI -DBOOST_ASIO_DISABLE_KQUEUE)

add_compile_options(-Wall)

if(DEFINED ENV{SLIC3R_VAR_REL})
    add_compile_options(-DVAR_REL=$ENV{SLIC3R_VAR_REL})
endif(DEFINED ENV{SLIC3R_VAR_REL})

if(DEFINED ENV{SLIC3R_VAR_ABS})
    add_compile_options(-DVAR_ABS)
endif(DEFINED ENV{SLIC3R_VAR_ABS})

if(DEFINED ENV{SLIC3R_VAR_ABS_PATH})
    add_compile_options(-DVAR_ABS_PATH=$ENV{SLIC3R_VAR_ABS_PATH})
endif(DEFINED ENV{SLIC3R_VAR_ABS_PATH})


execute_process(COMMAND git rev-parse --short HEAD OUTPUT_VARIABLE GIT_VERSION ERROR_QUIET)
if (CMAKE_BUILD_TYPE MATCHES Debug)
   add_compile_options(-g -O0)
elseif (CMAKE_BUILD_TYPE MATCHES Release)
   add_compile_options(-O3)
elseif (CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
   add_compile_options(-g -O3)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)


find_package(Threads REQUIRED)

set(Boost_NO_BOOST_CMAKE ON)
find_package(Boost REQUIRED COMPONENTS system thread filesystem)


if (NOT (${Boost_VERSION_STRING} VERSION_LESS "1.74.0"))
    MESSAGE("Adding in boost::nowide")
    find_package(Boost REQUIRED COMPONENTS system thread filesystem OPTIONAL_COMPONENTS nowide)
endif()

set(LIBDIR ${CMAKE_CURRENT_SOURCE_DIR}/../xs/src/)

# directory that contains the dependent non-source files, like models and configurations
set(TESTFILE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test/inputs/)

set(EXPAT_INCLUDES
    ${LIBDIR}/expat
)
set(ADMESH_INCLUDES
    ${LIBDIR}/admesh
)

set(BSPLINE_INCLUDES
    ${LIBDIR}/BSpline/
)

set(POLY2TRI_INCLUDES
    ${LIBDIR}/poly2tri
    ${LIBDIR}/poly2tri/sweep
    ${LIBDIR}/poly2tri/common
)
set(COMMON_INCLUDES
    ${LIBDIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/standalone/
)
set(SLIC3R_INCLUDES
    ${COMMON_INCLUDES}
    ${EXPAT_INCLUDES}
    ${ADMESH_INCLUDES}
    ${POLY2TRI_INCLUDES}
    ${BSPLINE_INCLUDES}
)
set(LIBSLIC3R_INCLUDES
    ${LIBDIR}/libslic3r
    ${LIBDIR}/libslic3r/IO
)


add_library(ZipArchive SHARED
    ${LIBDIR}/Zip/ZipArchive.cpp
)

add_library(miniz SHARED
    ${LIBDIR}/miniz/miniz.c
)
target_compile_features(ZipArchive PUBLIC cxx_std_11)
target_include_directories(ZipArchive PUBLIC ${COMMON_INCLUDES})
target_compile_options(ZipArchive PUBLIC -w)

add_library(libslic3r SHARED
    ${LIBDIR}/libslic3r/BoundingBox.cpp
    ${LIBDIR}/libslic3r/BridgeDetector.cpp
    ${LIBDIR}/libslic3r/ClipperUtils.cpp
    ${LIBDIR}/libslic3r/ConfigBase.cpp
    ${LIBDIR}/libslic3r/Config.cpp
    ${LIBDIR}/libslic3r/ConditionalGCode.cpp
    ${LIBDIR}/libslic3r/ExPolygon.cpp
    ${LIBDIR}/libslic3r/ExPolygonCollection.cpp
    ${LIBDIR}/libslic3r/Extruder.cpp
    ${LIBDIR}/libslic3r/ExtrusionEntity.cpp
    ${LIBDIR}/libslic3r/ExtrusionEntityCollection.cpp
    ${LIBDIR}/libslic3r/Fill/Fill.cpp
    ${LIBDIR}/libslic3r/Fill/Fill3DHoneycomb.cpp
    ${LIBDIR}/libslic3r/Fill/FillConcentric.cpp
    ${LIBDIR}/libslic3r/Fill/FillHoneycomb.cpp
    ${LIBDIR}/libslic3r/Fill/FillPlanePath.cpp
    ${LIBDIR}/libslic3r/Fill/FillRectilinear.cpp
    ${LIBDIR}/libslic3r/Fill/FillGyroid.cpp
    ${LIBDIR}/libslic3r/Flow.cpp
    ${LIBDIR}/libslic3r/GCode.cpp
    ${LIBDIR}/libslic3r/PrintGCode.cpp
    ${LIBDIR}/libslic3r/GCode/CoolingBuffer.cpp
    ${LIBDIR}/libslic3r/GCode/SpiralVase.cpp
    ${LIBDIR}/libslic3r/GCodeReader.cpp
    ${LIBDIR}/libslic3r/GCodeSender.cpp
    ${LIBDIR}/libslic3r/GCodeTimeEstimator.cpp
    ${LIBDIR}/libslic3r/GCodeWriter.cpp
    ${LIBDIR}/libslic3r/Geometry.cpp
    ${LIBDIR}/libslic3r/IO.cpp
    ${LIBDIR}/libslic3r/IO/AMF.cpp
    ${LIBDIR}/libslic3r/IO/TMF.cpp
    ${LIBDIR}/libslic3r/Layer.cpp
    ${LIBDIR}/libslic3r/LayerRegion.cpp
    ${LIBDIR}/libslic3r/LayerRegionFill.cpp
    ${LIBDIR}/libslic3r/LayerHeightSpline.cpp
    ${LIBDIR}/libslic3r/Line.cpp
    ${LIBDIR}/libslic3r/Log.cpp
    ${LIBDIR}/libslic3r/Model.cpp
    ${LIBDIR}/libslic3r/MotionPlanner.cpp
    ${LIBDIR}/libslic3r/MultiPoint.cpp
    ${LIBDIR}/libslic3r/PerimeterGenerator.cpp
    ${LIBDIR}/libslic3r/PlaceholderParser.cpp
    ${LIBDIR}/libslic3r/Point.cpp
    ${LIBDIR}/libslic3r/Polygon.cpp
    ${LIBDIR}/libslic3r/Polyline.cpp
    ${LIBDIR}/libslic3r/PolylineCollection.cpp
    ${LIBDIR}/libslic3r/Print.cpp
    ${LIBDIR}/libslic3r/PrintConfig.cpp
    ${LIBDIR}/libslic3r/PrintObject.cpp
    ${LIBDIR}/libslic3r/PrintRegion.cpp
    ${LIBDIR}/libslic3r/SimplePrint.cpp
    ${LIBDIR}/libslic3r/SLAPrint.cpp
    ${LIBDIR}/libslic3r/SlicingAdaptive.cpp
    ${LIBDIR}/libslic3r/Surface.cpp
    ${LIBDIR}/libslic3r/SurfaceCollection.cpp
    ${LIBDIR}/libslic3r/SVG.cpp
    ${LIBDIR}/libslic3r/TriangleMesh.cpp
    ${LIBDIR}/libslic3r/TransformationMatrix.cpp
    ${LIBDIR}/libslic3r/SupportMaterial.cpp
    ${LIBDIR}/libslic3r/utils.cpp
    ${LIBDIR}/libslic3r/miniz_extension.cpp
)
target_compile_features(libslic3r PUBLIC cxx_std_11)
target_include_directories(libslic3r SYSTEM PUBLIC ${SLIC3R_INCLUDES})
target_include_directories(libslic3r PUBLIC ${LIBSLIC3R_INCLUDES})
if (BOOST_NOWIDE_FOUND)
    target_compile_options(libslic3r -DBOOST_INCLUDE_NOWIDE)
endif()
target_link_libraries(libslic3r PUBLIC
    ${Boost_LIBRARIES}
    ${EXPAT_LIBRARIES}
    ${ADMESH_LIBRARIES}
    ${POLY2TRI_LIBRARIES}
    ${BSPLINE_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${CMAKE_DL_LIBS}
    ${CMAKE_REQUIRED_LIBRARIES}
    ${CMAKE_REQUIRED_LIBRARIES}
    ${CMAKE_REQUIRED})

add_library(BSpline SHARED ${LIBDIR}/BSpline/BSpline.cpp)
target_include_directories(BSpline PUBLIC ${BSPLINE_INCLUDES})
target_compile_options(BSpline PUBLIC -w)

add_library(admesh SHARED
    ${LIBDIR}/admesh/connect.c
    ${LIBDIR}/admesh/normals.c
    ${LIBDIR}/admesh/shared.c
    ${LIBDIR}/admesh/stl_io.c
    ${LIBDIR}/admesh/stlinit.c
    ${LIBDIR}/admesh/util.c
)
target_include_directories(admesh PUBLIC ${ADMESH_INCLUDES} ${COMMON_INCLUDES})
set_property(TARGET admesh PROPERTY C_STANDARD 99)
target_compile_options(admesh PUBLIC -w)


add_library(clipper SHARED ${LIBDIR}/clipper.cpp)
target_compile_features(clipper PUBLIC cxx_std_11)
target_include_directories(clipper PUBLIC ${COMMON_INCLUDES})
target_compile_options(clipper PUBLIC -w)

add_library(expat SHARED
    ${LIBDIR}/expat/xmlparse.c
    ${LIBDIR}/expat/xmlrole.c
    ${LIBDIR}/expat/xmltok.c
)
target_compile_features(expat PUBLIC cxx_std_11)
target_include_directories(expat PUBLIC ${EXPAT_INCLUDES})
target_compile_options(expat PUBLIC -w)

add_library(polypartition SHARED ${LIBDIR}/polypartition.cpp)
target_include_directories(polypartition PUBLIC ${COMMON_INCLUDES})
target_compile_options(polypartition PUBLIC -w)

add_library(poly2tri SHARED
    ${LIBDIR}/poly2tri/common/shapes.cc
    ${LIBDIR}/poly2tri/sweep/advancing_front.cc
    ${LIBDIR}/poly2tri/sweep/cdt.cc
    ${LIBDIR}/poly2tri/sweep/sweep_context.cc
    ${LIBDIR}/poly2tri/sweep/sweep.cc
)
target_include_directories(poly2tri PUBLIC ${COMMON_INCLUDES})
target_compile_options(poly2tri PUBLIC -w)

set(SLIC3R_TEST_SOURCES
    ${TESTDIR}/test_harness.cpp
    ${TESTDIR}/test_data.cpp
    ${TESTDIR}/libslic3r/test_config.cpp
    ${TESTDIR}/libslic3r/test_fill.cpp
    ${TESTDIR}/libslic3r/test_flow.cpp
    ${TESTDIR}/libslic3r/test_gcodewriter.cpp
    ${TESTDIR}/libslic3r/test_gcode.cpp
    ${TESTDIR}/libslic3r/test_geometry.cpp
    ${TESTDIR}/libslic3r/test_log.cpp
    ${TESTDIR}/libslic3r/test_model.cpp
    ${TESTDIR}/libslic3r/test_polygon.cpp
    ${TESTDIR}/libslic3r/test_print.cpp
    ${TESTDIR}/libslic3r/test_printgcode.cpp
    ${TESTDIR}/libslic3r/test_printobject.cpp
    ${TESTDIR}/libslic3r/test_skirt_brim.cpp
    ${TESTDIR}/libslic3r/test_test_data.cpp
    ${TESTDIR}/libslic3r/test_transformationmatrix.cpp
    ${TESTDIR}/libslic3r/test_trianglemesh.cpp
    ${TESTDIR}/libslic3r/test_extrusion_entity.cpp
    ${TESTDIR}/libslic3r/test_3mf.cpp
    ${TESTDIR}/libslic3r/test_amf.cpp
)



if (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} VERSION_GREATER 3.10)
cmake_policy(SET CMP0072 NEW)
endif (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} VERSION_GREATER 3.10)


find_library(bsystem_l boost_system log)
add_library(bsystem SHARED IMPORTED)
set_target_properties(bsystem PROPERTIES IMPORTED_LOCATION ${bsystem_l})
find_library(bthread_l boost_thread)
add_library(bthread SHARED IMPORTED)
set_target_properties(bthread PROPERTIES IMPORTED_LOCATION ${bthread_l})
include_directories(${Boost_INCLUDE_DIRS})

# Libraries that Libslic3r itself depends on.
set(LIBSLIC3R_DEPENDS
    admesh
    BSpline
    clipper
    expat
    polypartition
    poly2tri
    ZipArchive
    miniz
    ${Boost_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT} 
)


# export the library liblibslic3r.so
export(
    TARGETS 
        libslic3r 
        admesh
        BSpline
        clipper
        expat
        polypartition
        poly2tri
        ZipArchive
        miniz
    FILE 
        ${CMAKE_BINARY_DIR}/libslic3r.cmake)
