#include "bvh.h"

#include <numeric>
#include <iostream>

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
    return (t.vertices[0] + t.vertices[1] + t.vertices[2]) * (1.0f/3.0f);
  });

  _nodes.reserve(triangles.size()*2 - 1);
  _root = createNode(0, triangles.size());
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
    const std::array<float, 3> bbox_dims = {
      node->bbox.max.x - centroid_bbox.min.x,
      centroid_bbox.max.y - centroid_bbox.min.y,
      centroid_bbox.max.z - centroid_bbox.min.z
    };

    const std::array<Plane, 3> split_planes = {
      Plane({ centroid_bbox.min.x + (bbox_dims[0] / 2.0f), 0.0f, 0.0f }, { 1.0f , 0.0f, 0.0f }),
      Plane({ 0.0f, centroid_bbox.min.y + (bbox_dims[1] / 2.0f), 0.0f }, { 0.0f , 1.0f, 0.0f }),
      Plane({ 0.0f, 0.0f, centroid_bbox.min.z + (bbox_dims[2] / 2.0f) }, { 0.0f , 0.0f, 1.0f }),
    };

    // Try to splitNode along largest node bbox dimension first
    int split_dim = std::distance(bbox_dims.begin(), std::max_element(bbox_dims.begin(), bbox_dims.end()));
    Plane split_plane = split_planes[split_dim];

    // Try splitting until a split is found that has triangles at both sides of the splitNode plane. If
    // one side is empty, move to the next bbox dimensions. If none of the XYZ dimensions work, the
    // node triangles can not be splitNode (?)
    std::optional<int> split_index;

    for (int i = 0; !split_index && (i < 3); i++)
    {
      split_index = splitNode(from, to, split_plane);

      if (!split_index)
      {
        split_dim = (split_dim + 1) % 3;
        split_plane = split_planes[split_dim];
      }
    }

    if (split_index)
    {
      node->left = createNode(from, *split_index);
      node->right = createNode(*split_index, to);
    }
    else
    {
      _failed = true;
    }
  }

  return node;
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
