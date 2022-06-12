cmake_minimum_required (VERSION 3.0)

# Project & build system configuration
project (flatrace VERSION 0.0.1)

if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE "Debug")
endif ()

set (CMAKE_OSX_DEPLOYMENT_TARGET 12.0)

set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

# Compiler configuration
set (CMAKE_CXX_STANDARD 17)

if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
  if (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    add_compile_options (-Wall -Wno-unused-function -Ofast -DFMT_HEADER_ONLY -DSIMDE_ENABLE_NATIVE_ALIASES -fno-tree-vectorize -fno-tree-slp-vectorize)
  endif ()

  if (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    add_compile_options (-Wall -Wno-unused-function -Ofast -DFMT_HEADER_ONLY -DSIMDE_ENABLE_NATIVE_ALIASES -fno-vectorize -fno-tree-vectorize -fno-slp-vectorize -fno-tree-slp-vectorize)
  endif ()
else ()
  add_compile_options (-g -Wall -O0 -DFMT_HEADER_ONLY -DSIMDE_ENABLE_NATIVE_ALIASES)
endif ()


find_package (Boost 1.37.0 REQUIRED)
find_package (Threads REQUIRED)
find_package (GTest REQUIRED)

find_path (simde_INCLUDE_DIR simde/check.h REQUIRED)

include_directories (PRIVATE ".")
include_directories (PRIVATE "third_party")
include_directories (PRIVATE ${Boost_INCLUDE_DIRS})

add_library (core
  core/bvh.h
  core/bvh.cpp
  core/frame.h
  core/intersect.h
  core/types.h
)

add_library (utils
  utils/obj.cpp
  utils/obj.h
  utils/ppm.cpp
  utils/ppm.h
)

add_executable (flatrace main.cpp)

target_link_libraries (flatrace
  core
  utils
  Threads::Threads
)

include (CTest)
enable_testing ()

add_executable(tests
  test/main.cpp
  test/test_bvh.cpp
)

target_link_libraries (tests
  core
  utils
  Threads::Threads
  GTest::GTest
)

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/test)
file(CREATE_LINK ${CMAKE_SOURCE_DIR}/test/input ${CMAKE_BINARY_DIR}/test/input SYMBOLIC)

