// Implements a bounding volume hierarchy class for accelerated ray/triangle intersections.
//
// This code is based on the article 'How to build a BVH [2] & [3] by Jacco Bikker
//
// [2] https://jacco.ompf2.com/2022/04/18/how-to-build-a-bvh-part-2-faster-rays/
// [3] https://jacco.ompf2.com/2022/04/21/how-to-build-a-bvh-part-3-quick-builds/

#pragma once

#include "types.h"
#include "intersect.h"

#include <vector>
#include <optional>

namespace core {

class BVH
{
  public:
    BVH(const std::vector<Triangle> &triangles);

    bool intersect(Ray &ray, const int maxIntersections) const;
    bool intersect4x4(Ray4x4 &rays, const int maxIntersections) const;

    bool failed() const { return _failed; }

    const Triangle &getTriangle(const int i) const { return _triangles[_triangleIds[i]]; };
    const Vec3 &getCentroid(const int i) const { return _triangleCentroids[_triangleIds[i]]; };

    // return the root bounding box
    const BoundingBox& getRootBBox() const { return _root->bbox; }

  private:
    struct Node
    {
      public:
        Node(const int from, const int count)
        :
          leftFrom(from), count(count)
        {
        }

        BoundingBox bbox;

        int leftFrom;
        int count;

        bool isLeaf() const { return (count != 0); }
    };

    struct SplitDim
    {
      Vec3 normal;
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

    Node *splitNode(Node * const node);

    std::optional<int> partition(const int from, const int count, const Plane &splitPlane);
    std::optional<Plane> splitPlaneSAH(const Node * const node, const int from, const int count, const int maxSplitsPerDimension) const;

    void linearize();

    bool _failed;

    std::vector<Node> _nodes;
    std::vector<Triangle> _triangles;

    std::vector<int> _triangleIds;
    std::vector<Vec3> _triangleCentroids;

    Node *_root;

    int _maxDepth;
};



}
