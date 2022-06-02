#pragma once

#include "core/frame.h"

#include <string>

namespace utils::Ppm {

void write(const std::string &filename, const core::Frame &frame);

}
