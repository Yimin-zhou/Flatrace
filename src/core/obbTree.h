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
        Node(const int from, const int count, const uint32_t index)
                :
                leftFrom(from), count(count), index(index)
        {
        }

        uint32_t index;
        DiTO::OBB<float> obb;
        BoundingBox bbox;
        int leftFrom;
        int count;
        int groupNumber = -1;
        glm::vec3 cachedRayDir;

        bool isLeaf() const { return (count != 0); }
    };

    class ObbTree
    {
    public:
        ObbTree() = default;
        ObbTree(const std::vector<Triangle> &triangles, bool useSAH, bool useClustering, int num_clusters = 10);

        bool traversal(Ray &ray, const int maxIntersections, const std::vector<glm::vec3>& cachedClusterRaydirs, bool useRaycaching);
        bool traversal4x4(Ray4x4 &rays, const int maxIntersections) const;

        bool failed() const { return _failed; }
        const Triangle &getTriangle(const int i) const { return _triangles[_triangleIds[i]]; };
        const glm::vec3 &getCentroid(const int i) const { return _triangleCentroids[_triangleIds[i]]; };
        const Node *getRoot() const { return _root; }
        std::vector<Node> &getNodes() { return _nodes; }
        int getMaxDepth() const { return _maxDepth; }
        std::vector<int> getLeafDepths() const { return m_leafDepths; }

        template<typename F>
        void computeOBB(Node *node);

        // For grouping similar OBBs
        std::vector<std::vector<core::obb::Node>> clusterOBBsKmeans(int num_clusters);
        std::vector<std::vector<core::obb::Node>> clusterOBBsMeanshift();
        void cacheTransformations();
        std::vector<glm::mat4x4>& getTransformationCache() { return m_transformationCache; }

        // For cluster obb visualization
        std::vector<DiTO::OBB<float>> getClusterOBBs() const { return m_clusterOBBs; }

    private:
        struct SplitDim
        {
            glm::vec3 normal;
            float min;
            float max;
        };

        struct SplitBin
        {
            std::vector<DiTO::Vector<float>> obbBoundVertices;
            DiTO::OBB<float> bound;
//            Aabb aabb;
            int triangleCount = 0;
        };

        Node *splitNode(Node *const node, bool useSAH);
        std::optional<int> partition(const int from, const int count, const Plane &splitPlane);
        std::optional<Plane> splitPlaneMid(const Node *const node, int maxSplitsPerDimension) const;
        std::optional<Plane> splitPlaneSAH(const Node *const node, const int from, const int count, int maxSplitsPerDimension) const;
        void linearize();

        int calculateMaxLeafDepth(const Node *node, int depth = 1) const;
        int calculateMinLeafDepth(const Node *node, int depth = 1) const;
        void collectLeafDepths(const Node *node, int currentDepth = 1);

        // Ray intersection
        void triangleIntersection(const core::obb::Node *const node, core::Ray &ray);
        void intersectInternalNodes(const Node *node, core::Ray &ray, float& outT, const std::vector<glm::vec3>& cachedClusterRaydirs, bool useRaycaching);

        // SAH
        float evaluateSAH(const Node* const node, const glm::vec3& axis, const float candidateProj ) const;

        std::vector<int> m_leafDepths;

        bool _failed;
        std::vector<Node> _nodes;
        std::vector<Triangle> _triangles;
        std::vector<int> _triangleIds;
        std::vector<glm::vec3> _triangleCentroids;
        Node *_root;
        int _maxDepth;
        BoundingBox _unitAABB;

        // group similar nodes
        int m_nGroup;
        std::vector<glm::mat4x4> m_transformationCache;
        std::vector<std::vector<Node>> m_clusteredNodes;
        bool m_useClustering;

        // for visualization
        std::vector<DiTO::OBB<float>> m_clusterOBBs;
    };
}


