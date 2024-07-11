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
#include "src/utils/globalState.h"
#include "Tracy.hpp"

#ifdef IS_X86

#include <immintrin.h>

#else
#include <simde/x86/avx2.h>
#endif

namespace core
{

    BVH::BVH(const std::vector<Triangle> &triangles, bool useOBB, float offset)
            :
            _failed(false),
            _triangles(triangles),
            _triangleIds(triangles.size()),
            _triangleCentroids(triangles.size()),
            _unitAABB(glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(0.5f, 0.5f, 0.5f)),
            m_offset(offset)
    {
        std::iota(_triangleIds.begin(), _triangleIds.end(), 0);

        std::transform(triangles.begin(), triangles.end(), _triangleCentroids.begin(), [](const Triangle &t)
        {
            return (t.vertices[0] + t.vertices[1] + t.vertices[2]) / 3.0f;
        });

        _nodes.reserve(triangles.size() * 2 - 1);
        _root = &_nodes.emplace_back(0, triangles.size());

        splitNode(_root, useOBB);

        _maxDepth = calculateMaxLeafDepth(_root);
        int minDepth = calculateMinLeafDepth(_root);
        collectLeafDepths(_root);

//        std::cerr << "NODE STRUCT SIZE: " << sizeof(Node) << std::endl;
//        std::cerr << "BVH SIZE: " << _nodes.size() << std::endl;
//        std::cerr << "BVH MAX DEPTH: " << _maxDepth << std::endl;
//        std::cerr << "BVH MIN DEPTH: " << minDepth << std::endl;

        // Re-order triangles such that triangles for each node are adjacent in memory again. This should improve
        // data locality and avoids having to use indirection when iterating triangles for intersection
        linearize();
    }

// Linearize triangle indices
    void BVH::linearize()
    {
        std::vector<Triangle> linearized_triangles;
        std::vector<int> linearized_triangle_ids;

        for (const int triangle_id: _triangleIds)
        {
            linearized_triangle_ids.push_back(linearized_triangles.size());
            linearized_triangles.push_back(_triangles[triangle_id]);
        }

        std::swap(_triangleIds, linearized_triangle_ids);
        std::swap(_triangles, linearized_triangles);
    }


    int  BVH::calculateMaxLeafDepth(const Node* node, int depth) const
    {
        if (node->isLeaf())
        {
            return depth;
        }

        int leftDepth = calculateMaxLeafDepth(&_nodes[node->leftFrom], depth + 1);
        int rightDepth = calculateMaxLeafDepth(&_nodes[node->leftFrom + 1], depth + 1);

        return std::max(leftDepth, rightDepth);
    }

    int  BVH::calculateMinLeafDepth(const Node *node, int depth) const
    {
        if (node->isLeaf())
        {
            return depth;
        }

        int leftDepth = calculateMinLeafDepth(&_nodes[node->leftFrom], depth + 1);
        int rightDepth = calculateMinLeafDepth(&_nodes[node->leftFrom + 1], depth + 1);

        return std::min(leftDepth, rightDepth);
    }

    void  BVH::collectLeafDepths(const Node *node, int currentDepth)
    {
        if (node->isLeaf())
        {
            m_leafDepths.push_back(currentDepth);
            return;
        }

        collectLeafDepths(&_nodes[node->leftFrom], currentDepth + 1);
        collectLeafDepths(&_nodes[node->leftFrom + 1], currentDepth + 1);
    }


    bool BVH::traversal(Ray &ray, const int maxIntersections)
    {
        ZoneScopedN("AABB BVH Traversal");

        const Node *node_stack[_nodes.size()];

        if (core::intersectAABB(_root->bbox, ray) == INF)
        {
            return false;
        }

        for (int i = 0; i < maxIntersections; i++)
        {
            int stack_pointer = 0;

            node_stack[stack_pointer++] = _root;

            while (stack_pointer != 0)
            {
                const Node *const node = node_stack[--stack_pointer];

                ray.bvh_nodes_visited++;
                if (node->isLeaf())
                {
                    triangleIntersection(node, ray);
                }
                else
                {
                    const Node *left = &_nodes[node->leftFrom];
                    const Node *right = left + 1;

                    float t_left = 0;
                    float t_right = 0;

                    intersectInternalNodesAABB(left, ray, t_left);
                    intersectInternalNodesAABB(right, ray, t_right);

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

    bool BVH::traversal4x4(Ray4x4 &rays, const int maxIntersections) const
    {
        static const __m256 inf_x8 = _mm256_set1_ps(INF);

        const Node *node_stack[_nodes.size()];

        bool hit = false;
        bool dead = false;

        for (int i = 0; !dead && (i < maxIntersections); i++)
        {
            int stack_pointer = 0;

            node_stack[stack_pointer++] = _root;

            while (stack_pointer != 0)
            {
                const Node *const node = node_stack[--stack_pointer];

                if (node->isLeaf())
                {
                    for (int i = node->leftFrom; i < (node->leftFrom + node->count); i++)
                    {
                        core::intersect4x4(_triangles[i], rays);
                    }
                } else
                {
                    const Node *child_0 = &_nodes[node->leftFrom];
                    const Node *child_1 = child_0 + 1;

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
                    _mm256_cmp_ps(_mm256_load_ps(rays.t.data() + rays.n * 16), inf_x8, _CMP_NEQ_OQ),
                    _mm256_cmp_ps(_mm256_load_ps(rays.t.data() + rays.n * 16 + 8), inf_x8, _CMP_NEQ_OQ));

            dead = _mm256_testz_ps(h, h);

            if (!dead)
            {
                rays.nextIntersection();
                hit = true;
            }
        }
        return hit;
    }

    bool BVH::traversalOBB(Ray &ray, const int maxIntersections, bool useCaching) const
    {
        ZoneScopedN("OBB in AABB BVH Traversal");

        const Node *node_stack[_nodes.size()];

        if (core::intersectAABB(_root->bbox, ray) == INF)
        {
            return false;
        }

        for (int i = 0; i < maxIntersections; i++)
        {
            int stack_pointer = 0;

            node_stack[stack_pointer++] = _root;

            while (stack_pointer != 0)
            {
                const Node *const node = node_stack[--stack_pointer];

                ray.bvh_nodes_visited++;
                if (node->isLeaf())
                {

                    for (int j = node->leftFrom; j < (node->leftFrom + node->count); j++)
                    {
                        core::intersect(_triangles[j], ray);
                    }
                } else
                {
                    ZoneScopedN("Internal OBB Intersect");

                    const Node *left = &_nodes[node->leftFrom];
                    const Node *right = left + 1;

                    // transform ray to obb space for both left and right node
                    float t_left;
                    float t_right;
                    intersectInternalNodesOBB(left, ray, t_left, useCaching);
                    intersectInternalNodesOBB(right, ray, t_right, useCaching);

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

    bool BVH::traversalHybrid(Ray &ray, const int maxIntersections, bool useCaching)
    {
        ZoneScopedN("Hybrid BVH Traversal");

        const Node *node_stack[_nodes.size()];

        if (core::intersectAABB(_root->bbox, ray) == INF)
        {
            return false;
        }

        for (int i = 0; i < maxIntersections; i++)
        {
            int stack_pointer = 0;

            node_stack[stack_pointer++] = _root;

            while (stack_pointer != 0)
            {
                const Node *const node = node_stack[--stack_pointer];

                ray.bvh_nodes_visited++;
                if (node->isLeaf())
                {
                    triangleIntersection(node, ray);
                }
                else
                {
                    const Node *left = &_nodes[node->leftFrom];
                    const Node *right = left + 1;

                    float t_left = 0;
                    float t_right = 0;

                    if (left->obbFlag)
                    {
                        intersectInternalNodesOBB(left, ray, t_left, useCaching);
                    } else
                    {
                        intersectInternalNodesAABB(left, ray, t_left);
                    }

                    if (right->obbFlag)
                    {
                        intersectInternalNodesOBB(right, ray, t_right, useCaching);
                    } else
                    {
                        intersectInternalNodesAABB(right, ray, t_right);
                    }

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

    Node *BVH::splitNode(Node *const node, bool useOBB)
    {
        // generate obb
        if (useOBB)
        {
            computeOBB<float>(node);
        }

        // Calculate node bounding box
        for (int i = node->leftFrom; i < (node->leftFrom + node->count); i++)
        {
            for (const glm::vec3 &v: getTriangle(i).vertices)
            {
                node->bbox.min = glm::min(node->bbox.min, v);
                node->bbox.max = glm::max(node->bbox.max, v);
            }
        }

        // Compare surface area of AABB and OBB
        if (useOBB)
        {
            if (node->bbox.area() > node->obb.area() + m_offset)
            {
                node->obbFlag = true;
            }
        }

        // Subdivide if this is not a leaf node (getTriangle count below cutoff)
        if (node->count > LEAF_SIZE)
        {
            const std::optional<Plane> split_plane = splitPlaneSAH(node, node->leftFrom, node->count, 64);

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

                    splitNode(&_nodes[left_index], useOBB);
                    splitNode(&_nodes[right_index], useOBB);

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
    std::optional<Plane>
    BVH::splitPlaneSAH(const Node *const node, const int from, const int count, int maxSplitsPerDimension) const
    {
        const std::array<SplitDim, 3> split_dims =
                {
                SplitDim{{1.0f, 0.0f, 0.0f}, node->bbox.min.x, node->bbox.max.x},
                SplitDim{{0.0f, 1.0f, 0.0f}, node->bbox.min.y, node->bbox.max.y},
                SplitDim{{0.0f, 0.0f, 1.0f}, node->bbox.min.z, node->bbox.max.z},
        };

        const int splitsPerDimension = std::min(count, maxSplitsPerDimension);

        float best = INF;
        std::optional<Plane> best_plane;

        for (const SplitDim &split_dim: split_dims)
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
                const float cost_right = (n_right != 0 ? n_right *
                                                         (bins[plane_index].areaRight + bins[plane_index].bbox.area())
                                                       : INF);

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
            } else
            {
                std::swap(_triangleIds[left_to], _triangleIds[--right_from]);
            }
        }

        const int n_left = (left_to - from);
        const int n_right = count - n_left;

        return ((n_left != 0) && (n_right != 0) ? std::make_optional(left_to) : std::nullopt);
    }

    void BVH::intersectInternalNodesAABB(const Node *node, Ray &ray, float &outT)
    {
        outT = core::intersectAABB(node->bbox, ray);
    }

    void BVH::intersectInternalNodesOBB(const Node *node, Ray &ray, float &outT, bool useRaycaching) const
    {
        Ray tempRay = ray;
        if (useRaycaching)
        {
            glm::vec4 rayOriginalLocal = (node->obb.invMatrix) * glm::vec4(tempRay.o, 1.0f);
            tempRay.o = glm::vec3(rayOriginalLocal.x, rayOriginalLocal.y, rayOriginalLocal.z);
            tempRay.rd = glm::vec3(1.0f / node->cachedRayDir.x, 1.0f / node->cachedRayDir.y, 1.0f / node->cachedRayDir.z);
        } else
        {
            glm::vec4 rayOriginalLocal = (node->obb.invMatrix) * glm::vec4(tempRay.o, 1.0f);
            glm::vec4 rayDirectionLocal = (node->obb.invMatrix) * glm::vec4(tempRay.d, 0.0f);
            tempRay.o = glm::vec3(rayOriginalLocal.x, rayOriginalLocal.y, rayOriginalLocal.z);
            tempRay.rd = glm::vec3(1.0f / rayDirectionLocal.x, 1.0f / rayDirectionLocal.y,
                                   1.0f / rayDirectionLocal.z);
        }

        outT = core::intersectAABB(_unitAABB, tempRay);
    }

    void BVH::triangleIntersection(const core::Node *const node, Ray &ray)
    {
        for (int i = node->leftFrom; i < (node->leftFrom + node->count); i++)
        {
            core::intersect(getTriangle(i), ray);
        }
    }

    template<typename F>
    void BVH::computeOBB(Node *node)
    {
        std::vector<DiTO::Vector<F>> vertices;
        for (int i = node->leftFrom; i < (node->leftFrom + node->count); ++i)
        {
            for (const auto &v: getTriangle(i).vertices)
            {
                DiTO::Vector<F> vertex(v.x, v.y, v.z);
                vertices.push_back(vertex);
            }
        }

        // Compute the OBB using the DiTO algorithm, if there are vertices present
        if (!vertices.empty())
        {

            DiTO::DiTO_14(vertices.data(), vertices.size(), node->obb);
        }

        node->obb.ext = DiTO::Vector<F>(node->obb.ext.x + 0.001f, node->obb.ext.y + 0.001f, node->obb.ext.z + 0.001f);

        // create obb matrix, transform unit AABB (-0.5 - 0.5) to obb space
        // Scale matrix
        glm::mat4 scaleMatrix = glm::scale(glm::mat4(1.0f), 2.0f * (glm::vec3(node->obb.ext.x, node->obb.ext.y, node->obb.ext.z)));

        // Rotation matrix
        glm::mat4 rotationMatrix = glm::mat4(
                glm::vec4(glm::vec3(node->obb.v0.x, node->obb.v0.y, node->obb.v0.z), 0.0f),
                glm::vec4(glm::vec3(node->obb.v1.x, node->obb.v1.y, node->obb.v1.z), 0.0f),
                glm::vec4(glm::vec3(node->obb.v2.x, node->obb.v2.y, node->obb.v2.z), 0.0f),
                glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)
        );

        // Translation matrix
        glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(node->obb.mid.x, node->obb.mid.y, node->obb.mid.z));

        // Calculate the inverse of the transformation matrix
        node->obb.invMatrix = glm::inverse(translationMatrix * rotationMatrix * scaleMatrix);
    }
}