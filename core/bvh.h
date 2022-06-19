// Implements a bounding volume hierarchy class for accelerated ray/triangle intersections.
//
// This code is based on the article 'How to build a BVH - part 2' by Jacco Bikker

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

    bool intersect(Ray &ray) const;
    int intersect2x2(Ray2x2 &rays) const;
    int intersect4x4(Ray4x4 &rays) const;

    bool failed() const { return _failed; }

    const Triangle &getTriangle(const int i) const { return _triangles[_triangleIds[i]]; };
    const Vec3 &getCentroid(const int i) const { return _triangleCentroids[_triangleIds[i]]; };

  private:
    struct Node
    {
      Node(const int from, const int to)
      :
        from(from), to(to), isLeaf((to - from) <= 2), left(nullptr), right(nullptr)
      {
      }

      int depth() const
      {
        if (isLeaf)
        {
          return 1;
        }
        else
        {
          const int left_depth = (left != nullptr ? left->depth() : 0);
          const int right_depth = (right != nullptr ? right->depth() : 0);

          return 1 + std::max(left_depth, right_depth);
        }
      }

      int from;
      int to;
      bool isLeaf;

      Node *left;
      Node *right;
      BoundingBox bbox;
    };

    Node *createNode(const int from, const int to);
    std::optional<int> splitNode(const int from, const int to, const Plane &splitPlane);

    std::optional<Plane> splitPlaneSAH(const Node * const node, const int from, const int to, const int splitsPerDimension) const;

    bool _failed;

    std::vector<Node> _nodes;
    std::vector<Triangle> _triangles;

    std::vector<int> _triangleIds;
    std::vector<Vec3> _triangleCentroids;

    Node *_root;
    int _depth;
};



}
