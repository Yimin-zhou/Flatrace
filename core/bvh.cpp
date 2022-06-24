// Implements a bounding volume hierarchy class for accelerated ray/triangle intersections.
//
// This code is based on the article 'How to build a BVH [2] & [3] by Jacco Bikker
//
// [2] https://jacco.ompf2.com/2022/04/18/how-to-build-a-bvh-part-2-faster-rays/
// [3] https://jacco.ompf2.com/2022/04/21/how-to-build-a-bvh-part-3-quick-builds/

#include "bvh.h"

#include <numeric>
#include <iostream>
#include <cmath>

#include <simde/x86/avx2.h>

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
  _maxDepth = static_cast<int>(std::ceil(std::log2(_nodes.size())));
}

bool BVH::intersect(Ray &ray) const
{
  Node *node_stack[2 * _maxDepth];
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

int BVH::intersect4x4(Ray4x4 &rays) const
{
  Node *node_stack[2 * _maxDepth];
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

    const std::optional<Plane> split_plane = splitPlaneSAH(node, from, to, 32);

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
// bin all triangles and sum their area. The split positions for which the surface area heuristic
// C = (n_left * area_left) + (n_right * area_right) is lowest will be chosen.
std::optional<Plane> BVH::splitPlaneSAH(const Node * const node, const int from, const int to, int maxSplitsPerDimension) const
{
  const std::array<SplitDim, 3> split_dims = {
    SplitDim{ { 1.0f, 0.0f, 0.0f }, node->bbox.min.x, node->bbox.max.x },
    SplitDim{ { 0.0f, 1.0f, 0.0f }, node->bbox.min.y, node->bbox.max.y },
    SplitDim{ { 0.0f, 0.0f, 1.0f }, node->bbox.min.z, node->bbox.max.z },
  };

  const int splitsPerDimension = std::min(to - from, maxSplitsPerDimension);


  float best = INF;
  std::optional<Plane> best_plane;

  for (const SplitDim &split_dim : split_dims)
  {
    std::vector<SplitBin> bins(splitsPerDimension + 1);

    const float dim_width = (split_dim.max - split_dim.min);
    const float bin_width = dim_width / bins.size();

    // If all triangles in the current node lie in the same axis-aligned plane, 1 of the
    // split dimensions will have zero width (degenerate) and needs to be skipped. Similarly,
    // if the bin width drops below a (somewhat arbitrary) low threshold, binning and splitting
    // along this dimension is worthless and may lead to precision/roundoff errors, so we 
    // drop it.
    if (bin_width <= 1e-6)
    {
      continue;
    }

    // First bin all triangles and track the bin bounding boxes
    for (int triangle_index = from; triangle_index < to; triangle_index++)
    {
      const Vec3 &triangle_centroid = getCentroid(triangle_index);
      const float triangle_bin_offset = triangle_centroid.dot(split_dim.normal) - split_dim.min;
      const int bin_index = std::min<int>(triangle_bin_offset / bin_width, bins.size() - 1);

      bins[bin_index].bbox = bins[bin_index].bbox.extended(getTriangle(triangle_index));
      bins[bin_index].trianglesIn++;
    }

    // Now calculate the 'left of' and 'right of' bounding box area and # of triangles for each bin
    float left_sum = 0.0f;
    float right_sum = 0.0f;

    BoundingBox left_box;
    BoundingBox right_box;

    for (int i = 0; i < splitsPerDimension; i++)
    {
      const int bin_index_left = i;
      const int bin_index_right = bins.size() - i - 1;

      SplitBin &bin_left = bins[bin_index_left];
      SplitBin &bin_right = bins[bin_index_right];

      left_sum += bin_left.trianglesIn;
      left_box = left_box.extended(bin_left.bbox);

      bin_left.trianglesLeft = left_sum;
      bin_left.areaLeft = left_box.area();

      right_sum += bin_right.trianglesIn;
      right_box = right_box.extended(bin_right.bbox);

      bins[bin_index_right - 1].trianglesRight = right_sum;
      bins[bin_index_right - 1].areaRight = right_box.area();
    }

    // Now find the bin with the minimum cost. Note that for N bins we have (N -1) candidate planes,
    // so we skip bin in, and use the leftmost extent of each bins as the corresponding split plane offset.
    for (int plane_index = 1; plane_index < splitsPerDimension; plane_index++)
    {
      const double d = split_dim.min + plane_index * bin_width;

      const int n_left = bins[plane_index].trianglesLeft;
      const int n_right = bins[plane_index].trianglesRight + bins[plane_index].trianglesIn;

      const float cost_left = (n_left != 0 ? n_left * bins[plane_index].areaLeft : INF);
      const float cost_right = (n_right != 0 ? n_right * (bins[plane_index].areaRight + bins[plane_index].bbox.area()) : INF);

      const float cost = cost_left + cost_right;

      if (cost < best)
      {
        best_plane = Plane(split_dim.normal * d, split_dim.normal);
        best = cost;
      }
    }
  }

  return best_plane;
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
