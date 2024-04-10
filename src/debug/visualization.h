#pragma once

#include <vector>

#include "src/core/types.h"
#include "src/core/bvh.h"

namespace debug
{
    class Visualization
    {
    public:
        Visualization() = default;
        Visualization(const core::BVH &bvh) : m_bvh(bvh) {}
        // use BVH bounding boxes to construct the triangle objects
        core::BVH generateBbox();

    private:
        core::BVH m_bvh;

        void traversalNodes(const core::Node* node, std::vector<core::Triangle> &triangles, int &triangleId);

        std::vector<core::Triangle> visualizeAABB(const glm::vec3 &center,
                                                  const glm::vec3 &dimensions, int &triangleId) const;
        std::vector<core::Triangle> visualizeOBB(const DiTO::OBB &obb, int &triangleId) const;

    };
}
