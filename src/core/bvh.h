// Implements a bounding volume hierarchy class for accelerated ray/triangle intersections.
//
// This code is based on the article 'How to build a BVH [2] & [3] by Jacco Bikker
//
// [2] https://jacco.ompf2.com/2022/04/18/how-to-build-a-bvh-part-2-faster-rays/
// [3] https://jacco.ompf2.com/2022/04/21/how-to-build-a-bvh-part-3-quick-builds/

#pragma once

#include "types.h"
#include "intersect.h"
#include "dito/dito.h"
#include "imgui/imgui.h"

#include <vector>
#include <optional>

namespace core
{

    struct Node
    {
    public:
        Node(const int from, const int count)
                :
                leftFrom(from), count(count)
        {
        }

        BoundingBox bbox;
        DiTO::OBB<float> obb;

        int leftFrom;
        int count;
        glm::vec3 cachedRayDir;

        bool isLeaf() const { return (count != 0); }
    };

    class BVH
    {
    public:
        BVH() = default;
        BVH(const std::vector<Triangle> &triangles, bool useOBB);

        bool traversal(Ray &ray, const int maxIntersections);

        bool traversal4x4(Ray4x4 &rays, const int maxIntersections) const;

        bool traversalOBB(Ray &ray, const int maxIntersections, bool useCaching) const;

        bool failed() const { return _failed; }

        const Triangle &getTriangle(const int i) const { return _triangles[_triangleIds[i]]; };

        const glm::vec3 &getCentroid(const int i) const { return _triangleCentroids[_triangleIds[i]]; };

        const Node *getRoot() const { return _root; }

        std::vector<Node> &getNodes() { return _nodes; }

        int getMaxDepth() const { return _maxDepth; }
        std::vector<int> getLeafDepths() const { return m_leafDepths; }

        // Generate obb
        template<typename F>
        void computeOBB(Node *node);

    private:
        struct SplitDim
        {
            glm::vec3 normal;
            double min = 0.0f;
            double max = 0.0f;
        };

        struct SplitBin
        {
            BoundingBox bbox;

            float areaLeft = 0.0f;
            float areaRight = 0.0f;

            int trianglesIn = 0;
            int trianglesLeft = 0;
            int trianglesRight = 0;
        };

        Node *splitNode(Node *const node, bool useOBB);

        std::optional<int> partition(const int from, const int count, const Plane &splitPlane);

        std::optional<Plane>
        splitPlaneSAH(const Node *const node, const int from, const int count, const int maxSplitsPerDimension) const;

        void linearize();

        int calculateMaxLeafDepth(const Node *node, int depth = 1) const;
        int calculateMinLeafDepth(const Node *node, int depth = 1) const;
        void collectLeafDepths(const Node *node, int currentDepth = 1);

        // Ray intersection
        void triangleIntersection(const core::Node *const node, core::Ray &ray);
        void intersectInternalNodes(const Node *left, const Node *right, core::Ray &ray, float& outLeft, float& outRight);

        void intersectInternalNodesOBB(const Node *node, core::Ray &ray, float& outT, bool useRaycaching) const;

        std::vector<int> m_leafDepths;

        bool _failed;

        std::vector<Node> _nodes;
        std::vector<Triangle> _triangles;

        std::vector<int> _triangleIds;
        std::vector<glm::vec3> _triangleCentroids;

        Node *_root;

        int _maxDepth;
        BoundingBox _unitAABB;

    };


}
