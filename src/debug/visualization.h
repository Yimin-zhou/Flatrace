#pragma once

//TODO Not fully implemented yet

#include <vector>

#include "src/core/types.h"
#include "src/core/bvh.h"

namespace debug {
    class Visualization {
    public:
        Visualization();

        // use BVH bounding boxes to construct the triangle objects
//        std::vector<core::Triangle> visualize(const core::BVH &bvh);

    private:
//        std::vector<std::vector<core::Triangle>> _objects;
    };

    // Line debug object
    class Line : public Visualization{
    public:
        Line(const glm::vec3& start, const glm::vec3& end) : start(start), end(end) {}

        glm::vec3 start;
        glm::vec3 end;
    };

    class Box : public Visualization{
    public:
        Box(const core::BoundingBox& bbox);

        std::vector<Line> lines;
    };
}
