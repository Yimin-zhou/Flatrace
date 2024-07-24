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

        bool obbFlag;

        bool isLeaf() const { return (count != 0); }
    };

    class BVH
    {
    public:
        BVH() = default;

        BVH(const std::vector<Triangle> &triangles, bool useOBB, float offset = 0);

        bool traversal(Ray &ray, const int maxIntersections);

        bool traversal4x4(Ray4x4 &rays, const int maxIntersections) const;

        bool traversalOBB(Ray &ray, const int maxIntersections, bool useCaching) const;

        bool traversalHybrid(Ray &ray, const int maxIntersections, bool useCaching);

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

        void intersectInternalNodesAABB(const Node *node, Ray &ray, float &outT);

        void intersectInternalNodesOBB(const Node *node, core::Ray &ray, float &outT, bool useRaycaching) const;

        std::vector<int> m_leafDepths;

        bool m_failed;

        std::vector<Node> m_nodes;
        std::vector<Triangle> m_triangles;

        std::vector<int> m_triangleIds;
        std::vector<glm::vec3> m_triangleCentroids;

        Node *m_root;

        int m_maxDepth;
        BoundingBox m_unitAABB;

        float m_offset;

    };


}
