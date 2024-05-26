#pragma once

#include "types.h"
#include "intersect.h"
#include "dito/dito.h"
#include "imgui/imgui.h"

#include <vector>
#include <optional>
#include <numeric>

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
        glm::vec3 transformedRayDir;
        int leftFrom;
        int count;
        int groupNumber;

        bool isLeaf() const { return (count != 0); }
    };

    class ObbTree
    {
    public:
        ObbTree() = default;
        ObbTree(const std::vector<Triangle> &triangles, const glm::vec3 &rayDir);

        bool traversal(Ray &ray, const int maxIntersections);
        bool traversal4x4(Ray4x4 &rays, const int maxIntersections) const;

        bool failed() const { return _failed; }
        const Triangle &getTriangle(const int i) const { return _triangles[_triangleIds[i]]; };
        const glm::vec3 &getCentroid(const int i) const { return _triangleCentroids[_triangleIds[i]]; };
        const Node *getRoot() const { return _root; }
        const std::vector<Node> &getNodes() const { return _nodes; }
        int getMaxDepth() const { return _maxDepth; }
        std::vector<int> getLeafDepths() const { return m_leafDepths; }
        void setRayDir(const glm::vec3 &dir) { m_rayDir = dir; }

        template<typename F>
        void computeOBB(Node *node);

        // For grouping similar OBBs
        void preGenerateOBBs(int numOBBs); // TODO
        std::vector<std::vector<Node>> groupSimilarOBBs(float similarityThreshold);
        bool isSimilar(const DiTO::OBB<float>& obb1, const DiTO::OBB<float>& obb2, float similarityThreshold);
        void cacheTransformations();

        // For now just randomly group obbs
        std::vector<std::vector<Node>> groupRandomNodes(int numGroups);


    private:
        Node *splitNode(Node *const node);
        std::optional<int> partition(const int from, const int count, const Plane &splitPlane);
        std::optional<Plane> splitPlaneMid(const Node *const node, int maxSplitsPerDimension) const;
        std::optional<Plane> splitPlaneSAH(const Node *const node, int maxSplitsPerDimension) const;
        void linearize();

        int calculateMaxLeafDepth(const Node *node, int depth = 1) const;
        int calculateMinLeafDepth(const Node *node, int depth = 1) const;
        void collectLeafDepths(const Node *node, int currentDepth = 1);

        // Ray intersection
        void triangleIntersection(const core::obb::Node *const node, core::Ray &ray);
        void intersectInternalNodes(const Node *left, const Node *right, const core::obb::Node *const node, core::Ray &ray, float& outLeft, float& outRight);


        std::vector<int> m_leafDepths;

        glm::vec3 m_rayDir;
        bool _failed;
        std::vector<Node> _nodes;
        std::vector<Triangle> _triangles;
        std::vector<int> _triangleIds;
        std::vector<glm::vec3> _triangleCentroids;
        Node *_root;
        int _maxDepth;
        BoundingBox _unitAABB;

        // group similar nodes
        std::vector<DiTO::OBB<float>> m_preGeneratedOBBs;
        int m_nGroup;
        std::vector<glm::mat4x4> m_transformationCache;
        std::vector<std::vector<Node>> m_groupedNodes;
    };


}


