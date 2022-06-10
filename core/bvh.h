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
      Node *node_stack[_depth];
      int stack_pointer = 0;

      node_stack[stack_pointer++] = _root;

      while (stack_pointer != 0)
      {
        Node * const node = node_stack[--stack_pointer];

        if (node && core::intersect(node->bbox, ray))
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
            node_stack[stack_pointer++] = node->left;
            node_stack[stack_pointer++] = node->right;
          }
        }
      }

      return (ray.t != core::INF);
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

    bool _failed;

    std::vector<Node> _nodes;
    std::vector<Triangle> _triangles;

    std::vector<int> _triangleIds;
    std::vector<Vec3> _triangleCentroids;

    Node *_root;
    int _depth;
};



}
