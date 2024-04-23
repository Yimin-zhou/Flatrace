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
        ObbTree() = default;

        std::optional<Plane> splitPlaneOBB(const Node *const node, int maxSplitsPerDimension) const;
        void computeOBBPerObj(const std::vector<Triangle> &object, Node *node);

        // override functions
        void construtBVH(const std::vector<Triangle> &triangles) override;
        bool traversal(Ray &ray, const int maxIntersections) const override;
        bool traversalOBB(Ray &ray, const int maxIntersections) const override { traversal(ray, maxIntersections); };
        bool traversal4x4(Ray4x4 &rays, const int maxIntersections) const override;
        Node *splitNode(Node *const node) override;
        std::optional<int> partition(const int from, const int count, const Plane &splitPlane) override;
        void linearize() override;
        void computeOBB(Node *node) override;

        bool failed() const override { return _failed; }
        const Triangle &getTriangle(const int i) const override { return _triangles[_triangleIds[i]]; };
        const glm::vec3 &getCentroid(const int i) const override { return _triangleCentroids[_triangleIds[i]]; };
        const Node *getRoot() const override { return _root; }
        const std::vector<Node> &getNodes() const override { return _nodes; }
        int calculateMaxDepth(int index, int currentDepth = 0) override;
        int getMaxDepth() const override{ return _tempMaxDepth; }

    private:
        bool _failed;

        std::vector<Node> _nodes;
        std::vector<Triangle> _triangles;

        std::vector<int> _triangleIds;
        std::vector<glm::vec3> _triangleCentroids;

        Node *_root;

        int _tempMaxDepth = 0;

        unsigned int _nodeCount;
        BoundingBox _unitAABB;

        void init(const std::vector<Triangle> &triangles, unsigned int nodeIndex);
    };

}

