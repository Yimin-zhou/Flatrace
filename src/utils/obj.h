#pragma once

#include "src/core/types.h"

#include <vector>
#include <string>

namespace utils::Obj {

// old parser
std::vector<core::Triangle> read(const std::string &filename, const bool normalize = true);

// tinyobjloader parser
std::vector<core::Triangle> tinyRead(const std::string &filename);

void write_test_cubes(const std::string &filename);

}
