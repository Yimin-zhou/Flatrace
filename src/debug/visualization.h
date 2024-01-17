#pragma once

//TODO Not fully implemented yet

#include <vector>

#include "src/core/types.h"
#include "src/core/bvh.h"

namespace debug {
    class Visualization {
    public:
        // use BVH bounding boxes to construct the triangle objects
        std::vector<core::Triangle> visualizeBoundingBox(const core::BVH &bvh);
    };
}
