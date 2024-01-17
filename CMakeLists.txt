cmake_minimum_required (VERSION 3.0.2)

# Project & build system configuration
project (flatrace VERSION 0.0.1)

if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE "Debug")
endif ()

add_compile_options("$<$<CONFIG:DEBUG>:-DDEBUG>")
# add release flags
add_compile_options("$<$<CONFIG:RELEASE>:-DNDEBUG>")

set (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/site_cmake")

set (CMAKE_OSX_DEPLOYMENT_TARGET 12.0)

set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

# Compiler configuration
set (CMAKE_CXX_STANDARD 17)

add_compile_options(-g -Wall -Wno-unused-function -DFMT_HEADER_ONLY -DSIMDE_ENABLE_NATIVE_ALIASES)

if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES "i386")
  add_compile_options(-DIS_X86 -mavx2)
endif ()

if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
  if (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    add_compile_options (-Ofast -ffast-math -fno-finite-math-only -march=native -fno-tree-vectorize -fno-tree-slp-vectorize)
  endif ()

  if (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    add_compile_options (-Ofast -ffast-math -fno-finite-math-only -fno-vectorize -fno-tree-vectorize -fno-slp-vectorize -fno-tree-slp-vectorize)
  endif ()
elseif (${CMAKE_BUILD_TYPE} STREQUAL "Profile")
  if (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    add_compile_options (-O2 -ffast-math -march=native -fno-tree-vectorize -fno-tree-slp-vectorize)
  endif ()

  if (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    add_compile_options (-O2 -fno-vectorize -fno-tree-vectorize -fno-slp-vectorize -fno-tree-slp-vectorize)
  endif ()

else ()
  add_compile_options (-O0)
endif ()

find_package(glm REQUIRED)
find_package (Boost 1.37.0 REQUIRED)
find_package (TBB REQUIRED)
find_package (SDL2 2.0.17 REQUIRED)
find_package (GTest REQUIRED)

# find_path (simde_INCLUDE_DIR simde/check.h REQUIRED)
set (simde_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/simde/simde")

include_directories (PRIVATE ".")
include_directories (PRIVATE "third_party")
include_directories (PRIVATE ${Boost_INCLUDE_DIRS})
include_directories (PRIVATE ${SDL2_INCLUDE_DIRS})
include_directories (PRIVATE ${TBB_INCLUDE_DIR})
include_directories (PRIVATE "third_party/parser/tinyobjloader")

add_subdirectory (third_party/imgui)

add_library (core
        src/core/bvh.h
        src/core/bvh.cpp
        src/core/frame.h
        src/core/intersect.h
        src/core/types.h
        src/core/trace.h
)


add_library (utils
        src/utils/obj.cpp
        src/utils/obj.h
        src/utils/ppm.cpp
        src/utils/ppm.h
)

add_library(debug
        src/debug/visualization.cpp
        src/debug/visualization.h
        src/debug/bvh_debugger.h
)

add_executable (flatrace main.cpp)

target_link_libraries (flatrace
        core
        utils
#        debug
        imgui
        ${SDL2_LIBRARIES}
        ${TBB_LIBRARIES}
)

target_include_directories(flatrace PRIVATE ${GLM_INCLUDE_DIRS})
include (CTest)
enable_testing ()

add_executable(tests
  test/main.cpp
  test/test_flatrace.cpp
)

target_link_libraries (tests
        core
        utils
  GTest::GTest
)

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/test)
file(CREATE_LINK ${CMAKE_SOURCE_DIR}/test/input ${CMAKE_BINARY_DIR}/test/input SYMBOLIC)

