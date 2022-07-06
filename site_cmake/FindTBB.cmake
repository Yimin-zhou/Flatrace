# Find Intel TBB libraries.
#
# This uses the following environment variables to find the TBB distribution:
#
#   - TBB_ROOT
#
# Sets the following variables:
#
#   - TBB_FOUND
#   - TBB_LOCATION
#   - TBB_INCLUDE_DIR
#   - TBB_LIBRARIES

find_package (PackageHandleStandardArgs)

find_path (TBB_LOCATION include/tbb/tbb.h $ENV{TBB_ROOT})

if (TBB_LOCATION)
  find_library(TBB_LIBRARIES tbb PATHS ${TBB_LOCATION}/lib)
  set (TBB_INCLUDE_DIR ${TBB_LOCATION}/include)
endif ()

find_package_handle_standard_args (TBB DEFAULT_MSG TBB_INCLUDE_DIR TBB_LIBRARIES)

