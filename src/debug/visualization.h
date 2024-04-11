#pragma once

#include <vector>

#include "src/core/types.h"
#include "src/core/bvh/bvh.h"
#include "src/core/bvh/obbTree.h"


namespace debug
{
    class Visualization
    {
    public:
        Visualization() = default;

#if GEN_OBB_BVH
        Visualization(const core::ObbTree &bvh) : m_bvh(bvh) {}
#else
        Visualization(const core::BVH &bvh) : m_bvh(bvh) {}
#endif

        // use BVH bounding boxes to construct the triangle objects
        core::BVH generateBbox();

    private:
#if GEN_OBB_BVH
        core::ObbTree m_bvh;
#else
        core::BVH m_bvh;
#endif
        void traversalNodes(const core::Node* node, std::vector<core::Triangle> &triangles, int &triangleId);

        std::vector<core::Triangle> visualizeAABB(const glm::vec3 &center,
                                                  const glm::vec3 &dimensions, int &triangleId) const;
        std::vector<core::Triangle> visualizeOBB(const DiTO::OBB &obb, int &triangleId) const;

    };
}
