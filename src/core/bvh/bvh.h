// Implements a bounding volume hierarchy class for accelerated ray/triangle intersections.
//
// This code is based on the article 'How to build a BVH [2] & [3] by Jacco Bikker
//
// [2] https://jacco.ompf2.com/2022/04/18/how-to-build-a-bvh-part-2-faster-rays/
// [3] https://jacco.ompf2.com/2022/04/21/how-to-build-a-bvh-part-3-quick-builds/

#pragma once

#include "src/core/types.h"
#include "src/core/intersect.h"
#include "src/core/dito/dito.h"
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
        DiTO::OBB obb;

        int leftFrom;
        int count;

        bool isLeaf() const { return (count != 0); }
    };

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


    class BVH
    {
    public:
        BVH(const std::vector<Triangle> &triangles);

        BVH() = default;

        virtual void construtBVH(const std::vector<Triangle> &triangles) = 0;
        virtual bool traversal(Ray &ray, const int maxIntersections) const = 0;
        virtual bool traversal4x4(Ray4x4 &rays, const int maxIntersections) const = 0;

        virtual Node *splitNode(Node *const node) = 0;
        virtual std::optional<int> partition(const int from, const int count, const Plane &splitPlane) = 0;
        virtual void linearize() = 0;
        // Generate obb
        virtual void computeOBB(Node *node) = 0;

        virtual bool failed() const = 0;
        virtual const Triangle &getTriangle(const int i) const  = 0;
        virtual const glm::vec3 &getCentroid(const int i) const  = 0;
        virtual const Node *getRoot() const  = 0;
        virtual const std::vector<Node> &getNodes() const  = 0;
        virtual int calculateMaxDepth(int index, int currentDepth = 0)  = 0;
        virtual int getMaxDepth() const = 0;
    };

}
