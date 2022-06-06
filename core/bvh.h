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

    bool intersect(Ray &ray) const
    {
      return intersectNode(_root, ray);
    }

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

      int from;
      int to;
      bool isLeaf;

      Node *left;
      Node *right;
      BoundingBox bbox;
    };

    Node *createNode(const int from, const int to);
    std::optional<int> splitNode(const int from, const int to, const Plane &splitPlane);

    bool intersectNode(const Node * const node, Ray &ray) const
    {
      if (core::intersect(node->bbox, ray))
      {
        if (node->isLeaf)
        {
          for (int i = node->from; i < node->to; i++)
          {
            core::intersect(getTriangle(i), ray);
          }
        }
        else
        {
          intersectNode(node->left, ray);
          intersectNode(node->right, ray);
        }
      }

      return !std::isinf(ray.t);
    }

    bool _failed;

    std::vector<Node> _nodes;
    std::vector<Triangle> _triangles;

    std::vector<int> _triangleIds;
    std::vector<Vec3> _triangleCentroids;

    Node *_root;
};



}
