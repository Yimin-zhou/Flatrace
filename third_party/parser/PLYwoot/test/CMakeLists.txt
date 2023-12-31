find_package(Boost COMPONENTS iostreams REQUIRED)
find_package(Catch2 3 REQUIRED)

# Main test executable; depends on Catch2, requires C++14 support.
add_executable(tests
        main.cpp
        test_istream.cpp
        test_ostream.cpp
        test_iostream.cpp
        test_type_traits.cpp
)
target_link_libraries(tests
  PRIVATE
  plywoot
  Boost::iostreams
  Catch2::Catch2WithMain
)

# TODO(ton): seems there is a bug in GCC 12 where it issues a warning in the
# standard library code for a trivial `std::reverse` call.
set_source_files_properties(test_istream.cpp PROPERTIES COMPILE_FLAGS -Wno-strict-overflow)

# Executables that are only compiled to test compatibility for the various C++
# standards.
add_executable(test-cpp11 test-cpp11.cpp)
target_link_libraries(test-cpp11
  PRIVATE
  plywoot
)
