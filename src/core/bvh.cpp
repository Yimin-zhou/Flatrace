// Implements a bounding volume hierarchy class for accelerated ray/triangle intersections.
//
// This code is based on the article 'How to build a BVH [2] & [3] by Jacco Bikker
//
// [2] https://jacco.ompf2.com/2022/04/18/how-to-build-a-bvh-part-2-faster-rays/
// [3] https://jacco.ompf2.com/2022/04/21/how-to-build-a-bvh-part-3-quick-builds/

#include "bvh.h"

#include "glm/glm.hpp"
#include <glm/gtc/matrix_transform.hpp>


#include <numeric>
#include <iostream>
#include <cmath>

#ifdef IS_X86
#include <immintrin.h>
#else
#include <simde/x86/avx2.h>
#endif

namespace core {

    BVH::BVH(const std::vector<Triangle> &triangles)
            :
            _failed(false),
            _triangles(triangles),
            _triangleIds(triangles.size()),
            _triangleCentroids(triangles.size()),
            _unitAABB(glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(0.5f, 0.5f, 0.5f))
    {
        initiateAABBBVH(triangles);
    }

    void BVH::initiateAABBBVH(const std::vector<Triangle> &triangles) {
        std::iota(_triangleIds.begin(), _triangleIds.end(), 0);

        std::transform(triangles.begin(), triangles.end(), _triangleCentroids.begin(), [](const Triangle &t)
        {
            return (t.vertices[0] + t.vertices[1] + t.vertices[2]) / 3.0f;
        });

        _nodes.reserve(triangles.size()*2 - 1);
        _root = &_nodes.emplace_back(0, triangles.size());

        // construct obb in this function
        splitNode(_root);

        _tempMaxDepth = calculateMaxDepth(0);

        std::cerr << "NODE STRUCT SIZE: " << sizeof(Node) << std::endl;
        std::cerr << "BVH SIZE: " << _nodes.size() << std::endl;
        std::cerr << "BVH MAX DEPTH: " << _tempMaxDepth << std::endl;

        // Re-order triangles such that triangles for each node are adjacent in memory again. This should improve
        // data locality and avoids having to use indirection when iterating triangles for intersection
        linearize();
    }

    void BVH::initiateOBBBVH(const std::vector<Triangle> &triangles) {
        std::iota(_triangleIds.begin(), _triangleIds.end(), 0);

        std::transform(triangles.begin(), triangles.end(), _triangleCentroids.begin(), [](const Triangle &t)
        {
            return (t.vertices[0] + t.vertices[1] + t.vertices[2]) / 3.0f;
        });

        _nodes.reserve(triangles.size()*2 - 1);
        _root = &_nodes.emplace_back(0, triangles.size());

        // construct obb in this function
        splitNode(_root);

        _tempMaxDepth = calculateMaxDepth(0);

        std::cerr << "NODE STRUCT SIZE: " << sizeof(Node) << std::endl;
        std::cerr << "BVH SIZE: " << _nodes.size() << std::endl;
        std::cerr << "BVH MAX DEPTH: " << _tempMaxDepth << std::endl;

        // Re-order triangles such that triangles for each node are adjacent in memory again. This should improve
        // data locality and avoids having to use indirection when iterating triangles for intersection
        linearize();
    }

    // Linearize triangle indices
    void BVH::linearize()
    {
        std::vector<Triangle> linearized_triangles;
        std::vector<int> linearized_triangle_ids;

        for (const int triangle_id : _triangleIds)
        {
            linearized_triangle_ids.push_back(linearized_triangles.size());
            linearized_triangles.push_back(_triangles[triangle_id]);
        }

        std::swap(_triangleIds, linearized_triangle_ids);
        std::swap(_triangles, linearized_triangles);
    }

    bool BVH::intersect(Ray &ray, const int maxIntersections) const
    {
        const Node *node_stack[2 * _tempMaxDepth];

        if (core::intersectAABB(_root->bbox, ray) == INF)
        {
            return false;
        }

        for (int i = 0; i < maxIntersections; i++)
        {
            int stack_pointer = 0;

            node_stack[stack_pointer++] = _root;

            // Traversal works like this: while there are nodes left on the stack, pop the topmost one. If it is a leaf,
            // intersect & shorten the ray against the triangles in the leaf node. If the node is an internal node,
            // intersect the ray against its left & right child node bboxes, and push those child nodes that were hit,
            // ordered by hit distance, to ensure the closest node gets traversed first.
            while (stack_pointer != 0)
            {
                const Node * const node = node_stack[--stack_pointer];

                ray.bvh_nodes_visited++;
                if (node->isLeaf())
                {
                    for (int i = node->leftFrom; i < (node->leftFrom + node->count); i++)
                    {
                        core::intersect(getTriangle(i), ray);
                    }
                }
                else
                {
                    const Node *left = &_nodes[node->leftFrom];
                    const Node *right = left + 1;

                    float t_left = core::intersectAABB(left->bbox, ray);
                    float t_right = core::intersectAABB(right->bbox, ray);

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

            ray.nextIntersection();
        }

        return (ray.t[0] != core::INF);
    }

    bool BVH::intersect4x4(Ray4x4 &rays, const int maxIntersections) const
    {
        static const __m256 inf_x8 = _mm256_set1_ps(INF);

        const Node *node_stack[2 * _tempMaxDepth];

        bool hit = false;
        bool dead = false;

        for (int i = 0; !dead && (i < maxIntersections); i++)
        {
            int stack_pointer = 0;

            node_stack[stack_pointer++] = _root;

            while (stack_pointer != 0)
            {
                const Node * const node = node_stack[--stack_pointer];


                if (node->isLeaf())
                {
                    for (int i = node->leftFrom; i < (node->leftFrom + node->count); i++)
                    {
                        core::intersect4x4(_triangles[i], rays);
                    }
                }
                else
                {
                    const Node * child_0 = &_nodes[node->leftFrom];
                    const Node * child_1 = child_0 + 1;

                    float t_0 = core::intersect4x4(child_0->bbox, rays);
                    float t_1 = core::intersect4x4(child_1->bbox, rays);

                    // Swap nodes based on shortest intersection distance. This reduces traversal time dependency
                    // on the camera direction, by shortening rays before traversing obscured nodes.
                    if (t_0 > t_1)
                    {
                        std::swap(t_0, t_1);
                        std::swap(child_0, child_1);
                    }

                    if (t_0 != INF)
                    {
                        if (t_1 != INF)
                        {
                            node_stack[stack_pointer++] = child_1;
                        }

                        node_stack[stack_pointer++] = child_0;
                    }
                }
            }

            const __m256 h = _mm256_or_ps(
                    _mm256_cmp_ps(_mm256_load_ps(rays.t.data() + rays.n*16), inf_x8, _CMP_NEQ_OQ),
                    _mm256_cmp_ps(_mm256_load_ps(rays.t.data() + rays.n*16 + 8), inf_x8, _CMP_NEQ_OQ));

            dead = _mm256_testz_ps(h, h);

            if (!dead)
            {
                rays.nextIntersection();
                hit = true;
            }
        }
        return hit;
    }

    bool BVH::intersectObbBVH(Ray &ray, const int maxIntersections) const
    {
        const Node *node_stack[2 * _tempMaxDepth];

        if (core::intersectAABB(_root->bbox, ray) == INF)
        {
            return false;
        }

        for (int i = 0; i < maxIntersections; i++)
        {
            int stack_pointer = 0;

            node_stack[stack_pointer++] = _root;

            // Traversal works like this: while there are nodes left on the stack, pop the topmost one. If it is a leaf,
            // intersect & shorten the ray against the triangles in the leaf node. If the node is an internal node,
            // intersect the ray against its left & right child node bboxes, and push those child nodes that were hit,
            // ordered by hit distance, to ensure the closest node gets traversed first.
            while (stack_pointer != 0)
            {
                const Node * const node = node_stack[--stack_pointer];

                    ray.bvh_nodes_visited++;
                if (node->isLeaf())
                {
                    for (int j = node->leftFrom; j < (node->leftFrom + node->count); j++)
                    {
                        core::intersect(getTriangle(j), ray);
                    }
                }
                else
                {
                    const Node *left = &_nodes[node->leftFrom];
                    const Node *right = left + 1;

                     // transform ray to obb space for both left and right node
                    Ray leftRay = ray;
                    glm::vec4 rayOriginalLocal = (left->obb.invMatrix) * glm::vec4(leftRay.o, 1.0f);
                    glm::vec4 rayDirectionLocal = (left->obb.invMatrix) * glm::vec4(leftRay.d, 0.0f);
                    leftRay.o = glm::vec3(rayOriginalLocal.x, rayOriginalLocal.y, rayOriginalLocal.z);
                    leftRay.rd = glm::vec3(1.0f / rayDirectionLocal.x, 1.0f / rayDirectionLocal.y, 1.0f / rayDirectionLocal.z);

                    Ray rightRay = ray;
                    rayOriginalLocal = (right->obb.invMatrix) * glm::vec4(rightRay.o, 1.0f);
                    rayDirectionLocal = (right->obb.invMatrix) * glm::vec4(rightRay.d, 0.0f);
                    rightRay.o = glm::vec3(rayOriginalLocal.x, rayOriginalLocal.y, rayOriginalLocal.z);
                    rightRay.rd = glm::vec3(1.0f / rayDirectionLocal.x, 1.0f / rayDirectionLocal.y, 1.0f / rayDirectionLocal.z);

                    float t_left = core::intersectAABB(_unitAABB, leftRay);
                    float t_right = core::intersectAABB(_unitAABB, rightRay);

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

            ray.nextIntersection();
        }

        return (ray.t[0] != core::INF);

    }

    Node* BVH::splitNodeOBB(Node* const node)
    {
        // generate obb
        computeOBB(node);

        // Calculate node bounding box, and getCentroid bounding box (for splitting)
        BoundingBox centroid_bbox;

        for (int i = node->leftFrom; i < (node->leftFrom + node->count); i++)
        {
            for (const glm::vec3 &v : getTriangle(i).vertices)
            {
                node->bbox.min = glm::min(node->bbox.min, v);
                node->bbox.max = glm::max(node->bbox.max, v);
            }

            centroid_bbox.min = glm::min(centroid_bbox.min, getCentroid(i));
            centroid_bbox.max = glm::max(centroid_bbox.max, getCentroid(i));
        }

        // Subdivide if this is not a leaf node (getTriangle count below cutoff)
        if (node->count > 1)
        {
            const std::optional<Plane> split_plane = splitPlaneSAHOBB(node, node->leftFrom, node->count, 32);

            if (split_plane)
            {
                const std::optional<int> split_index = partitionOBB(node->leftFrom, node->count, *split_plane);

                if (split_index)
                {
                    const int left_index = _nodes.size();
                    const int right_index = left_index + 1;

                    const int left_count = *split_index - node->leftFrom;
                    const int right_count = node->count - left_count;

                    _nodes.emplace_back(node->leftFrom, left_count);
                    _nodes.emplace_back(*split_index, right_count);

                    splitNodeOBB(&_nodes[left_index]);
                    splitNodeOBB(&_nodes[right_index]);

                    node->leftFrom = left_index;
                    node->count = 0;
                }
            }
        }

        return node;
    }


// Calculate split plane using surface area heuristic. This will pick a number of uniformly distributed
// split plane positions (specified by the splitsPerDimension parameter) for each XYZ dimension, then
// bin all triangles and sum their area. The split positions for which the surface area heuristic
// C = (n_left * area_left) + (n_right * area_right) is lowest will be chosen.
    std::optional<Plane> BVH::splitPlaneSAHOBB(const Node * const node, const int from, const int count, int maxSplitsPerDimension) const
    {
        const DiTO::OBB& obb = node->obb;
        std::optional<Plane> best_plane;
        float bestCost = INF;

        // Directions for OBB's local axes
        std::array<glm::dvec3, 3> axes = {obb.v0, obb.v1, obb.v2};

        for (int axis = 0; axis < 3; ++axis) {
            int splitsPerDimension = std::min(node->count, maxSplitsPerDimension);
            std::vector<SplitBin> bins(splitsPerDimension + 1, SplitBin{});

            float dimExtent = obb.ext[axis]; // Extent along the current axis
            float binWidth = (2 * dimExtent) / splitsPerDimension;

            if (binWidth <= 1e-6) continue; // Skip degenerate dimensions

            // Bin triangles based on their projection along the current OBB axis
            for (int i = 0; i < node->count; ++i) {
                const glm::vec3 centroid = getCentroid(i + from); // Assuming getCentroid computes the centroid of a triangle
                glm::dvec3 localPos = (glm::dvec3) centroid - obb.mid; // Position relative to OBB center
                float projection = glm::dot(localPos, axes[axis]); // Project onto the current axis
                float normalizedDist = projection + dimExtent; // Normalize distance to [0, 2*dimExtent]
                int binIndex = std::min(static_cast<int>(normalizedDist / binWidth), splitsPerDimension);

                // Update bins with triangle info (assuming bins track counts and bounding boxes)
                bins[binIndex].trianglesIn++;
                // Extend bin's bounding box with the triangle's AABB (requires implementation)
            }

            // Calculate SAH costs for each split position
            for (int splitIndex = 1; splitIndex < splitsPerDimension; ++splitIndex) {
                // Compute the cost of this split using SAH
                float costLeft = 0.0f, costRight = 0.0f;
                int countLeft = 0, countRight = 0;

                // Aggregate counts and costs for left and right sides
                for (int i = 0; i < splitIndex; ++i) {
                    countLeft += bins[i].trianglesIn;
                    // Calculate costLeft (requires implementation)
                }
                for (int i = splitIndex; i < splitsPerDimension; ++i) {
                    countRight += bins[i].trianglesIn;
                    // Calculate costRight (requires implementation)
                }

                float splitCost = costLeft + costRight; // Simplified; actual SAH formula needed

                if (splitCost < bestCost) {
                    bestCost = splitCost;
                    // Calculate plane position and normal based on OBB orientation and split position
                    glm::dvec3 planePos = obb.mid + axes[axis] * (glm::dvec3)(binWidth * splitIndex - dimExtent);
                    glm::dvec3 planeNormal = axes[axis];
                    best_plane = Plane{planePos, planeNormal};
                }
            }
        }

        return best_plane;
    }

// Partition getTriangle range [leftFrom, leftFrom + count) into a subset behind, and a subset in front of the passed
// split plane, and return the index of the resulting partition point
    std::optional<int> BVH::partitionOBB(const int from, const int count, const Plane &splitPlane)
    {
        int left_to = from;
        int right_from = from + count;

        while (left_to < right_from)
        {
            const glm::dvec3 c = getCentroid(left_to);

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
        const int n_right = count - n_left;

        return ((n_left != 0) && (n_right != 0) ? std::make_optional(left_to) : std::nullopt);
    }

    Node* BVH::splitNode(Node* const node)
    {
        // generate obb
        computeOBB(node);

        // Calculate node bounding box, and getCentroid bounding box (for splitting)
        BoundingBox centroid_bbox;

        for (int i = node->leftFrom; i < (node->leftFrom + node->count); i++)
        {
            for (const glm::vec3 &v : getTriangle(i).vertices)
            {
                node->bbox.min = glm::min(node->bbox.min, v);
                node->bbox.max = glm::max(node->bbox.max, v);
            }

            centroid_bbox.min = glm::min(centroid_bbox.min, getCentroid(i));
            centroid_bbox.max = glm::max(centroid_bbox.max, getCentroid(i));
        }

        // Subdivide if this is not a leaf node (getTriangle count below cutoff)
        if (node->count > 1)
        {
            const std::optional<Plane> split_plane = splitPlaneSAH(node, node->leftFrom, node->count, 32);

            if (split_plane)
            {
                const std::optional<int> split_index = partition(node->leftFrom, node->count, *split_plane);

                if (split_index)
                {
                    const int left_index = _nodes.size();
                    const int right_index = left_index + 1;

                    const int left_count = *split_index - node->leftFrom;
                    const int right_count = node->count - left_count;

                    _nodes.emplace_back(node->leftFrom, left_count);
                    _nodes.emplace_back(*split_index, right_count);

                    splitNode(&_nodes[left_index]);
                    splitNode(&_nodes[right_index]);

                    node->leftFrom = left_index;
                    node->count = 0;
                }
            }
        }

        return node;
    }


// Calculate split plane using surface area heuristic. This will pick a number of uniformly distributed
// split plane positions (specified by the splitsPerDimension parameter) for each XYZ dimension, then
// bin all triangles and sum their area. The split positions for which the surface area heuristic
// C = (n_left * area_left) + (n_right * area_right) is lowest will be chosen.
    std::optional<Plane> BVH::splitPlaneSAH(const Node * const node, const int from, const int count, int maxSplitsPerDimension) const
    {
        const std::array<SplitDim, 3> split_dims = {
                SplitDim{ { 1.0f, 0.0f, 0.0f }, node->bbox.min.x, node->bbox.max.x },
                SplitDim{ { 0.0f, 1.0f, 0.0f }, node->bbox.min.y, node->bbox.max.y },
                SplitDim{ { 0.0f, 0.0f, 1.0f }, node->bbox.min.z, node->bbox.max.z },
        };

        const int splitsPerDimension = std::min(count, maxSplitsPerDimension);

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
            for (int triangle_index = from; triangle_index < (from + count); triangle_index++)
            {
                const glm::vec3 &triangle_centroid = getCentroid(triangle_index);
                const float triangle_bin_offset = glm::dot(triangle_centroid, split_dim.normal) - split_dim.min;
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
                const float d = split_dim.min + plane_index * bin_width;

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

// Partition getTriangle range [leftFrom, leftFrom + count) into a subset behind, and a subset in front of the passed
// split plane, and return the index of the resulting partition point
    std::optional<int> BVH::partition(const int from, const int count, const Plane &splitPlane)
    {
        int left_to = from;
        int right_from = from + count;

        while (left_to < right_from)
        {
            const glm::dvec3 c = getCentroid(left_to);

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
        const int n_right = count - n_left;

        return ((n_left != 0) && (n_right != 0) ? std::make_optional(left_to) : std::nullopt);
    }

    // The following functions are used for visualizing the BVH in the debug viewer
    std::vector<Triangle> BVH::visualizeBVH() const {
        std::vector<Triangle> triangles;
        int triangleId = 0;
        visualizeNode(_root, triangles, triangleId);
        return triangles;
    }

    std::vector<Triangle> BVH::visualizeBVHOBB() const {
        std::vector<Triangle> triangles;
        int triangleId = 0;
        visualizeNodeOBB(_root, triangles, triangleId);
        return triangles;
    }

    void BVH::visualizeNode(const Node* node, std::vector<Triangle>& triangles, int& triangleId) const {
        if (node == nullptr) {
            return;
        }
    //     Only visualize the bounding box if it's a leaf node
       if (node->isLeaf()) {
            glm::vec3 center = (node->bbox.min + node->bbox.max) * 0.5f;
            glm::vec3 dimensions = node->bbox.max - node->bbox.min;
            std::vector<Triangle> nodeTriangles = visualizeAABB(center, dimensions, triangleId);
            triangles.insert(triangles.end(), nodeTriangles.begin(), nodeTriangles.end());
       } else {
            // Recurse for children
           visualizeNode(&_nodes[node->leftFrom], triangles, triangleId);
           visualizeNode(&_nodes[node->leftFrom + 1], triangles, triangleId);
       }
    }

    std::vector<core::Triangle> BVH::visualizeAABB(const glm::vec3& center, const glm::vec3& dimensions, int& triangleId) const{
        std::vector<Triangle> triangles;
        glm::vec3 halfDimensions = dimensions * 0.5f;

        // Calculate vertices based on center and dimensions
        std::array<glm::vec3, 8> vertices = {
                center + glm::vec3(-halfDimensions.x, -halfDimensions.y, -halfDimensions.z),
                center + glm::vec3(halfDimensions.x, -halfDimensions.y, -halfDimensions.z),
                center + glm::vec3(halfDimensions.x, halfDimensions.y, -halfDimensions.z),
                center + glm::vec3(-halfDimensions.x, halfDimensions.y, -halfDimensions.z),
                center + glm::vec3(-halfDimensions.x, -halfDimensions.y, halfDimensions.z),
                center + glm::vec3(halfDimensions.x, -halfDimensions.y, halfDimensions.z),
                center + glm::vec3(halfDimensions.x, halfDimensions.y, halfDimensions.z),
                center + glm::vec3(-halfDimensions.x, halfDimensions.y, halfDimensions.z)
        };

        std::array<std::array<int, 6>, 6> faces = {{
                                                           {0, 3, 2, 2, 1, 0}, // Front face
                                                           {1, 2, 6, 6, 5, 1}, // Right face
                                                           {5, 6, 7, 7, 4, 5}, // Back face
                                                           {4, 7, 3, 3, 0, 4}, // Left face
                                                           {4, 0, 1, 1, 5, 4}, // Bottom face
                                                           {3, 7, 6, 6, 2, 3}  // Top face
                                                   }};

        for (const auto& face : faces) {
            glm::vec3 v0 = vertices[face[0]];
            glm::vec3 v1 = vertices[face[1]];
            glm::vec3 v2 = vertices[face[2]];
            glm::vec3 v3 = vertices[face[3]];
            glm::vec3 v4 = vertices[face[4]];
            glm::vec3 v5 = vertices[face[5]];

            // Adjust the order of vertices to ensure normals are pointing outward
            triangles.emplace_back(triangleId++, v0, v1, v2, 0);
            triangles.emplace_back(triangleId++, v3, v4, v5, 0);
        }

        return triangles;
    }

    void BVH::visualizeNodeOBB(const Node *node, std::vector<Triangle> &triangles, int &triangleId) const
    {
        if (node == nullptr) {
            return;
        }
        // Only visualize the bounding box if it's a leaf node
        if (node->isLeaf()) {
            std::vector<Triangle> nodeTriangles = visualizeOBB(node->obb, triangleId);
            triangles.insert(triangles.end(), nodeTriangles.begin(), nodeTriangles.end());
        } else {
            // Recurse for children
            visualizeNodeOBB(&_nodes[node->leftFrom], triangles, triangleId);
            visualizeNodeOBB(&_nodes[node->leftFrom + 1], triangles, triangleId);
        }
    }

    std::vector<core::Triangle> BVH::visualizeOBB(const DiTO::OBB& obb, int& triangleId) const {
        std::vector<Triangle> triangles;

        // Convert OBB vectors to glm::vec3
        glm::vec3 mid = obb.mid;
        glm::vec3 v0 = obb.v0;
        glm::vec3 v1 = obb.v1;
        glm::vec3 v2 = obb.v2;
        glm::vec3 ext =obb.ext;

        // Calculate vertices of the OBB use ext
        std::array<glm::vec3, 8> vertices;
        vertices = {{
                           mid + v0 * ext.x + v1 * ext.y + v2 * ext.z,
                           mid - v0 * ext.x + v1 * ext.y + v2 * ext.z,
                           mid - v0 * ext.x - v1 * ext.y + v2 * ext.z,
                           mid + v0 * ext.x - v1 * ext.y + v2 * ext.z,
                           mid + v0 * ext.x + v1 * ext.y - v2 * ext.z,
                           mid - v0 * ext.x + v1 * ext.y - v2 * ext.z,
                           mid - v0 * ext.x - v1 * ext.y - v2 * ext.z,
                           mid + v0 * ext.x - v1 * ext.y - v2 * ext.z
                   }};

        std::array<std::array<int, 6>, 6> faces = {{
                                                           {0, 3, 2, 2, 1, 0}, // Front face
                                                           {1, 2, 6, 6, 5, 1}, // Right face
                                                           {5, 6, 7, 7, 4, 5}, // Back face
                                                           {4, 7, 3, 3, 0, 4}, // Left face
                                                           {4, 0, 1, 1, 5, 4}, // Bottom face
                                                           {3, 7, 6, 6, 2, 3}  // Top face
                                                   }};

        // Create triangles from vertices
        for (const auto& face : faces) {
            glm::vec3 v0 = vertices[face[0]];
            glm::vec3 v1 = vertices[face[1]];
            glm::vec3 v2 = vertices[face[2]];
            glm::vec3 v3 = vertices[face[3]];
            glm::vec3 v4 = vertices[face[4]];
            glm::vec3 v5 = vertices[face[5]];

            // Adjust the order of vertices to ensure normals are pointing inwards
            triangles.emplace_back(triangleId++, v2, v1, v0, 0);
            triangles.emplace_back(triangleId++, v5, v4, v3, 0);
        }

        return triangles;
    }

    int BVH::calculateMaxDepth(int index, int currentDepth) {
        if (index >= _nodes.size() || _nodes[index].isLeaf()) return currentDepth;

        // Assuming right child immediately follows left child in the nodes vector
        int leftDepth = calculateMaxDepth(_nodes[index].leftFrom, currentDepth + 1);
        int rightDepth = calculateMaxDepth(_nodes[index].leftFrom + 1, currentDepth + 1);

        return std::max(leftDepth, rightDepth);
    }

    void BVH::computeOBB(Node* node) {
        std::vector<glm::dvec3> vertices;
        for (int i = node->leftFrom; i < (node->leftFrom + node->count); ++i) {
            for (const glm::vec3 &v : getTriangle(i).vertices)
            {
                vertices.push_back(v);
            }
        }

        // Compute the OBB using the DiTO algorithm, if there are vertices present
        if (!vertices.empty()) {
            DiTO::DiTO_14(vertices.data(), vertices.size(), node->obb );
        }

        node->obb.ext = glm::max(node->obb.ext, glm::dvec3(0.001));

        // create obb matrix, transform unit AABB (-0.5 - 0.5) to obb space
        // Scale matrix
        glm::mat4 scaleMatrix = glm::scale(glm::mat4(1.0f), 2.0f * ( glm::vec3(node->obb.ext)));

        // Rotation matrix
        glm::mat4 rotationMatrix = glm::mat4(
                glm::vec4(glm::vec3(node->obb.v0), 0.0f),
                glm::vec4(glm::vec3(node->obb.v1), 0.0f),
                glm::vec4(glm::vec3(node->obb.v2), 0.0f),
                glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)
        );

        // Translation matrix
        glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(node->obb.mid));

        // Calculate the inverse of the transformation matrix
        node->obb.invMatrix = glm::inverse(translationMatrix * rotationMatrix * scaleMatrix);
    }
}