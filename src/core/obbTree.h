#pragma once

#include "types.h"
#include "intersect.h"
#include "dito/dito.h"
#include "imgui/imgui.h"

#include <vector>
#include <optional>

namespace core::obb
{
    struct Node
    {
    public:
        Node(const int from, const int count)
                :
                leftFrom(from), count(count)
        {
        }

        DiTO::OBB<float> obb;
//        BoundingBox bbox;

        int leftFrom;
        int count;

        bool isLeaf() const { return (count != 0); }
    };

    class ObbTree
    {
    public:
        ObbTree(const std::vector<Triangle> &triangles);

        bool traversal(Ray &ray, const int maxIntersections) const;

        bool traversal4x4(Ray4x4 &rays, const int maxIntersections) const;

        bool failed() const { return _failed; }
        const Triangle &getTriangle(const int i) const { return _triangles[_triangleIds[i]]; };
        const glm::vec3 &getCentroid(const int i) const { return _triangleCentroids[_triangleIds[i]]; };
        const Node *getRoot() const { return _root; }
        const std::vector<Node> &getNodes() const { return _nodes; }
        int getMaxDepth() const { return _maxDepth; }

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

        Node *splitNode(Node *const node);
        std::optional<int> partition(const int from, const int count, const Plane &splitPlane);
        std::optional<Plane> splitPlaneOBB(const Node *const node, int maxSplitsPerDimension) const;
        void linearize();

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


