#include "bvh.h"

#include <numeric>
#include <iostream>

#include <simde/x86/avx2.h>

#include <fmt/ostream.h>

namespace core {

BVH::BVH(const std::vector<Triangle> &triangles)
:
  _failed(false),
  _triangles(triangles),
  _triangleIds(triangles.size()),
  _triangleCentroids(triangles.size())
{
  std::iota(_triangleIds.begin(), _triangleIds.end(), 0);

  std::transform(triangles.begin(), triangles.end(), _triangleCentroids.begin(), [](const Triangle &t)
  {
    return (t.vertices[0] + t.vertices[1] + t.vertices[2]) / 3.0f;
  });

  _nodes.reserve(triangles.size()*2 - 1);
  _root = createNode(0, triangles.size());
  _depth = _root->depth();
}

bool BVH::intersect(Ray &ray) const
{
  Node *node_stack[_depth];
  int stack_pointer = 0;

  if (core::intersect(_root->bbox, ray) != INF)
  {
    node_stack[stack_pointer++] = _root;
  }

  // Traversal works like this: while there are nodes left on the stack, pop the topmost one. If it is a leaf,
  // intersect & shorten the ray against the triangles in the leaf node. If the node is an internal node,
  // intersect the ray against its left & right child node bboxes, and push those child nodes that were hit,
  // ordered by hit distance, to ensure the closest node gets traversed first.
  while (stack_pointer != 0)
  {
    Node * const node = node_stack[--stack_pointer];

    if (node->isLeaf)
    {
      for (int i = node->from; i < node->to; i++)
      {
        core::intersect(getTriangle(i), ray);
      }
    }
    else
    {
      Node *left = node->left;
      Node *right = node->right;

      float t_left = core::intersect(left->bbox, ray);
      float t_right = core::intersect(right->bbox, ray);

      if (t_left > t_right)
      {
        std::swap(t_left, t_right);
        std::swap(left, right);
      }

      if (t_left != INF)
      {
        if (t_right != INF)
        {
          node_stack[stack_pointer++] = right;
        }

        node_stack[stack_pointer++] = left;
      }
    }
  }

  return (ray.t != core::INF);
}

int BVH::intersect2x2(Ray2x2 &rays) const
{
  Node *node_stack[_depth];
  int stack_pointer = 0;

  node_stack[stack_pointer++] = _root;

  while (stack_pointer != 0)
  {
    Node * const node = node_stack[--stack_pointer];

    const int hit = core::intersect2x2(node->bbox, rays);

    if (hit)
    {
      if (node->isLeaf)
      {
        for (int i = node->from; i < node->to; i++)
        {
          core::intersect2x2(getTriangle(i), rays);
        }
      }
      else
      {
        node_stack[stack_pointer++] = node->left;
        node_stack[stack_pointer++] = node->right;
      }
    }
  }

  return _mm_movemask_ps(_mm_cmpneq_ps(_mm_load_ps(rays.t.data()), _mm_set1_ps(INF)));
}

int BVH::intersect4x4(Ray4x4 &rays) const
{
  Node *node_stack[_depth];
  int stack_pointer = 0;

  node_stack[stack_pointer++] = _root;

  while (stack_pointer != 0)
  {
    Node * const node = node_stack[--stack_pointer];

    const bool hit = core::intersect4x4(node->bbox, rays);

    if (hit)
    {
      if (node->isLeaf)
      {
        for (int i = node->from; i < node->to; i++)
        {
          core::intersect4x4(getTriangle(i), rays);
        }
      }
      else
      {
        node_stack[stack_pointer++] = node->left;
        node_stack[stack_pointer++] = node->right;
      }
    }
  }

  __m256 inf = _mm256_set1_ps(INF);

  return
    (_mm256_movemask_ps(_mm256_cmp_ps(_mm256_load_ps(rays.t.data()), inf, SIMDE_CMP_NEQ_OQ))) |
    (_mm256_movemask_ps(_mm256_cmp_ps(_mm256_load_ps(rays.t.data() + 8), inf, SIMDE_CMP_NEQ_OQ)) << 8);
}

BVH::Node *BVH::createNode(const int from, const int to)
{
  // Create new node
  Node * const node = &_nodes.emplace_back(from, to);

  if (from == to)
  {
    return node;
  }

  // Calculate node bounding box, and getCentroid bounding box (for splitting)
  BoundingBox centroid_bbox;

  for (int i = from; i < to ; i++)
  {
    for (const Vec3 &v : getTriangle(i).vertices)
    {
      node->bbox.min = Vec3::min(node->bbox.min, v);
      node->bbox.max = Vec3::max(node->bbox.max, v);
    }

    centroid_bbox.min = Vec3::min(centroid_bbox.min, getCentroid(i));
    centroid_bbox.max = Vec3::max(centroid_bbox.max, getCentroid(i));
  }

  // Subdivide if this is not a leaf node (getTriangle count below cutoff)
  if (!node->isLeaf)
  {
    bool have_split = false;

    const std::optional<Plane> split_plane = splitPlaneSAH(node, from, to, 8);

    if (split_plane)
    {
      const std::optional<int> split_index = splitNode(from, to, *split_plane);

      if (split_index)
      {
        node->left = createNode(from, *split_index);
        node->right = createNode(*split_index, to);
        have_split = true;
      }
    }

    node->isLeaf = !have_split;
  }

  return node;
}

// Calculate split plane using surface area heuristic. This will pick a number of uniformly distributed
// split plane positions (specified by the splitsPerDimension parameter) for each XYZ dimension, then
// bin all triangles and sum their area. The split positions for wich the surface area heuristic
// C = (n_left * area_left) + (n_right * area_right) is lowest will be chosen.
std::optional<Plane> BVH::splitPlaneSAH(const Node * const node, const int from, const int to, int splitsPerDimension) const
{
  struct SplitDim
  {
    Vec3 normal;
    double min;
    double max;
  };

  const std::array<SplitDim, 3> split_dims = {
    SplitDim{ { 1.0f, 0.0f, 0.0f }, node->bbox.min.x, node->bbox.max.x },
    SplitDim{ { 0.0f, 1.0f, 0.0f }, node->bbox.min.y, node->bbox.max.y },
    SplitDim{ { 0.0f, 0.0f, 1.0f }, node->bbox.min.z, node->bbox.max.z },
  };

  float best = INF;
  std::optional<Plane> best_plane;

  for (const SplitDim &split_dim : split_dims)
  {
    const double d = (split_dim.max - split_dim.min) / (splitsPerDimension + 1);

    for (int plane_i = 1; plane_i <= splitsPerDimension; plane_i++)
    {
      const Plane candidate_plane = { { split_dim.normal * (split_dim.min + d * plane_i) }, split_dim.normal };

      BoundingBox left_bbox = { { INF, INF, INF }, { -INF, -INF, -INF } };
      BoundingBox right_bbox = { { INF, INF, INF }, { -INF, -INF, -INF } };

      int n_left = 0;
      int n_right = 0;

      for (int i = from; i < to; i++)
      {
        const Triangle &triangle = getTriangle(i);
        const Vec3 &centroid = getCentroid(i);

        if (candidate_plane.distance(centroid) < 0.0f)
        {
          left_bbox = left_bbox.extended(triangle);
          n_left++;
        }
        else
        {
          right_bbox = right_bbox.extended(triangle);
          n_right++;
        }
      }

      const bool have_split = (n_left != 0) && (n_right != 0);
      const float c = (have_split ? n_left * left_bbox.area() + n_right * right_bbox.area() : INF);

      if (c < best)
      {
        best_plane = candidate_plane;
        best = c;
      }
    }
  }

  return *best_plane;
}

// Partition getTriangle range [from, to) into a subset behind, and a subset in front of the passed
// split plane, and return the index of the resulting splitNode point
std::optional<int> BVH::splitNode(const int from, const int to, const Plane &splitPlane)
{
  int left_to = from;
  int right_from = to;

  while (left_to < right_from)
  {
    const Vec3 c = getCentroid(left_to);

    if (splitPlane.distance(c) < 0.0f)
    {
      left_to++;
    }
    else
    {
      std::swap(_triangleIds[left_to], _triangleIds[--right_from]);
    }
  }

  const int n_left = (left_to - from);
  const int n_right = (to - right_from);

  return ((n_left != 0) && (n_right != 0) ? std::make_optional(left_to) : std::nullopt);
}

}
