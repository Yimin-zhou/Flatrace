#pragma once

#include <vector>

#include "src/core/types.h"
#include "src/core/bvh.h"
#include "src/core/obbTree.h"
#include "src/debug/visualization.h"


namespace debug
{
    class Visualization
    {
    public:
        Visualization() = default;
        Visualization(const core::BVH &bvh);
        Visualization(const core::obb::ObbTree &obbTree);

        std::vector<core::Triangle> getTriangles() const { return m_triangles; }

    private:
        core::BVH m_inputBVH;
        core::obb::ObbTree m_inputObbTree;

        std::vector<core::Triangle> m_triangles;

        void traversalNodes(const core::Node* node,
                            std::vector<core::Triangle> &triangles, int &triangleId);
        void traversalNodes(const core::obb::Node* node,
                            std::vector<core::Triangle> &triangles, int &triangleId);

        std::vector<core::Triangle> visualizeAABB(const glm::vec3 &center,
                                                  const glm::vec3 &dimensions, int &triangleId) const;

        template<typename T>
        std::vector<core::Triangle> visualizeOBB(const DiTO::OBB<T> &obb, int &triangleId) const;

    };
}
