#pragma once

#include "core/types.h"

#include <vector>
#include <string>

namespace utils::Obj {

std::vector<core::Triangle> read(const std::string &filename, const bool normalize = true);

}
