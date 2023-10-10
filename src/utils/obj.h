#pragma once

#include "src/core/types.h"

#include <vector>
#include <string>

namespace utils::Obj {

std::vector<core::Triangle> read(const std::string &filename, const bool normalize = true);

void write_test_cubes(const std::string &filename);

}
