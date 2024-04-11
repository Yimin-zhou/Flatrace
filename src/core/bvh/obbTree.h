#pragma once

#include "src/core/types.h"
#include "src/core/intersect.h"
#include "src/core/dito/dito.h"
#include "imgui/imgui.h"
#include "bvh.h"

#include <vector>
#include <optional>

namespace core
{

    class ObbTree : public BVH
    {
    public:
        ObbTree(const std::vector<std::vector<Triangle>> &objects);
        ObbTree(const std::vector<Triangle> &object);

        ObbTree() = default;

        void construtBVH(const std::vector<Triangle> &triangles) override;

        bool traversal(Ray &ray, const int maxIntersections) const override;

    protected:
        Node *splitNode(Node *const node) override;

        std::optional<int> partition(const int from, const int count,
                                     const Plane &splitPlane) override;

        std::optional<Plane> splitPlaneOBB(const Node *const node, int maxSplitsPerDimension) const;

        void linearize() override;

        void computeOBBPerObj(const std::vector<Triangle> &object, Node *node);

        unsigned int _nodeCount;

        BoundingBox _unitAABB;

        void init(const std::vector<Triangle> &triangles, unsigned int nodeIndex);
    };

}

