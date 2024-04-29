#pragma once

#include "core/bvh/bvh.h"

namespace core
{
    class AABBTree : public BVH
    {
    public:
        AABBTree() = default;
        AABBTree(const std::vector<Triangle> &triangles);

        std::optional<Plane>
        splitPlaneSAH(const Node *const node, const int from, const int count, const int maxSplitsPerDimension) const;

        // override functions
        void construtBVH(const std::vector<Triangle> &triangles) override;
        bool traversal(Ray &ray, const int maxIntersections) const override;
        bool traversalOBB(Ray &ray, const int maxIntersections) const override;
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
        int getMaxDepth() const override{ return _maxDepth; }

    private:
        bool _failed;

        std::vector<Node> _nodes;
        std::vector<Triangle> _triangles;

        std::vector<int> _triangleIds;
        std::vector<glm::vec3> _triangleCentroids;

        Node *_root;

        int _maxDepth;
        // We define this standard AABB space as a unit cube centered at the origin: Pmin = [–0.5, –0.5, –0.5], Pmax = [0.5, 0.5, 0.5]
        BoundingBox _unitAABB;
    };
}