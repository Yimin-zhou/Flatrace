#pragma once

#include <vector>

#include "src/core/types.h"
#include "src/core/bvh/bvh.h"
#include "src/core/bvh/aabbTree.h"
#include "src/core/bvh/obbTree.h"


namespace debug
{
    class Visualization
    {
    public:
        Visualization() = default;
        Visualization(const std::shared_ptr<core::BVH> &bvh);

        std::vector<core::Triangle> getTriangles() const { return m_triangles; }

    private:
        std::shared_ptr<core::BVH> m_inputBVH;

        std::vector<core::Triangle> m_triangles;

        void traversalNodes(const core::Node* node,
                            std::vector<core::Triangle> &triangles, int &triangleId);

        std::vector<core::Triangle> visualizeAABB(const glm::vec3 &center,
                                                  const glm::vec3 &dimensions, int &triangleId) const;
        std::vector<core::Triangle> visualizeOBB(const DiTO::OBB &obb, int &triangleId) const;

    };
}
