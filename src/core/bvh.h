#pragma once

#include "types.h"
#include "intersect.h"
#include "dito/dito.h"
#include "imgui/imgui.h"

#include <vector>
#include <optional>
#include <tbb/concurrent_hash_map.h>


namespace core
{

    struct Node
    {
    public:
        Node(const int from, const int count, const uint32_t index)
                :
                index(index), leftFrom(from), count(count)
        {
        }
        uint32_t index;
        BoundingBox bbox;
        DiTO::OBB<float> obb;

        int leftFrom;
        int count;
        int groupNumber = -1;

        bool obbFlag;

        bool isLeaf() const { return (count != 0); }
    };

    class BVH
    {
    public:
        BVH() = default;

        BVH(const std::vector<Triangle> &triangles, bool useOBB, bool isHybrid, int binSize = 100, bool useClustering = false, int nGroup = 10);

        bool traversal(Ray &ray, const int maxIntersections);

        bool traversal4x4(Ray4x4 &rays, const int maxIntersections) const;

        bool traversalOBB(Ray &ray, const int maxIntersections);

        bool traversalHybrid(Ray &ray, const int maxIntersections);

        bool failed() const { return m_failed; }

        const Triangle &getTriangle(const int i) const { return m_triangles[m_triangleIds[i]]; };

        const glm::vec3 &getCentroid(const int i) const { return m_triangleCentroids[m_triangleIds[i]]; };

        const Node *getRoot() const { return m_root; }

        std::vector<Node> &getNodes() { return m_nodes; }

        int getMaxDepth() const { return m_maxDepth; }

        std::vector<int> getLeafDepths() const { return m_leafDepths; }

        // Generate obb
        template<typename F>
        void computeOBB(Node *node);

        // Clustering
        std::vector<glm::mat4x4> getTransformationCache() { return m_transformationCache; }
        int getOBBLeafSize() { return m_enabledOBBLeafSize; }

        void clearRayDirCache() { m_cachedClusterRaydirs = std::vector<glm::vec3>(m_nGroup); }

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

        std::optional<Plane>
        splitPlaneSAH(const Node *const node, const int from, const int count, const int maxSplitsPerDimension) const;

        void linearize();

        int calculateMaxLeafDepth(const Node *node, int depth = 1) const;

        int calculateMinLeafDepth(const Node *node, int depth = 1) const;

        void collectLeafDepths(const Node *node, int currentDepth = 1);

        // Ray intersection
        void triangleIntersection(const core::Node *const node, core::Ray &ray);

        void intersectInternalNodesAABB(const Node *node, Ray &ray, float &outT);

        void intersectInternalNodesOBB(const Node *node, core::Ray &ray, float &outT);

        // Clustering for OBB
        std::vector<std::vector<Node>> clusterOBBsKmeans(int num_clusters);
        void cacheTransformations();

        int m_nGroup;
        std::vector<glm::mat4x4> m_transformationCache;
        std::vector<std::vector<Node>> m_clusteredNodes;
        bool m_useClustering;
        int m_enabledOBBLeafSize;

        std::vector<int> m_leafDepths;

        bool m_failed;

        bool m_useOBB;
        int m_binSize;

        std::vector<Node> m_nodes;
        std::vector<Triangle> m_triangles;

        std::vector<int> m_triangleIds;
        std::vector<glm::vec3> m_triangleCentroids;

        Node *m_root;

        int m_maxDepth;
        BoundingBox m_unitAABB;

        // Hybrid
        bool m_isHybrid;

        // for visualization
        std::vector<DiTO::OBB<float>> m_clusterOBBs;
        std::vector<glm::vec3> m_cachedClusterRaydirs;



    };


}
