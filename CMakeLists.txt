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
  add_compile_options (-Wall -O3 -DFMT_HEADER_ONLY -DSIMDE_ENABLE_NATIVE_ALIASES -fno-vectorize -fno-tree-vectorize -fno-slp-vectorize -fno-tree-slp-vectorize)
else ()
  add_compile_options ("-g" "-Wall" "-O0" "-DFMT_HEADER_ONLY" "-DSIMDE_ENABLE_NATIVE_ALIASES")
endif ()

if (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
endif ()

if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
endif ()

find_package (Boost 1.37.0 REQUIRED)
find_package (Threads REQUIRED)
find_package (GTest REQUIRED)

find_path (simde_INCLUDE_DIR simde/check.h REQUIRED)

include_directories (PRIVATE ".")
include_directories (PRIVATE "third_party")
include_directories (PRIVATE ${Boost_INCLUDE_DIRS})

add_library (core INTERFACE)
target_include_directories (core INTERFACE core)

add_library (utils
  utils/obj.cpp
  utils/obj.h
  utils/ppm.cpp
  utils/ppm.h
)

#add_subdirectory (test)

add_executable (flatrace main.cpp)

target_link_libraries (flatrace
  core
  utils
  Threads::Threads
)








