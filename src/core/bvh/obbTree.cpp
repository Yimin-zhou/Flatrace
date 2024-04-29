#include <Tracy.hpp>
#include "obbTree.h"

#include "src/utils/globalState.h"

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

namespace core
{
    ObbTree::ObbTree(const std::vector<std::vector<Triangle>> &objects) :
            _nodeCount(0),
            _unitAABB(glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(0.5f, 0.5f, 0.5f))
    {
#if OBB_METHOD_1
        _nodes.emplace_back(0, 0);
        for (const auto &object: objects)
        {
            _triangles.insert(_triangles.end(), object.begin(), object.end());
            init(object, ++_nodeCount);
            computeOBBPerObj(object, &_nodes[_nodeCount - 1]);
        }
        _root = &_nodes[0];
#else
        // add all triangles from all objects
        for (int i = 0; i < objects.size(); i++)
        {
            _triangles.insert(_triangles.end(), objects[i].begin(), objects[i].end());
        }
        _triangleIds.resize(_triangles.size());
        _triangleCentroids.resize(_triangles.size());
        construtBVH(_triangles);
#endif
    }

    void ObbTree::init(const std::vector<Triangle> &triangles, unsigned int nodeIndex)
    {
        _nodes.emplace_back(nodeIndex, triangles.size());
    }

    bool ObbTree::traversal(Ray &ray, const int maxIntersections) const
    {
        ZoneScopedN("OBB Traversal");
        const Node *node_stack[_nodes.size()];

        for (int i = 0; i < maxIntersections; i++)
        {
            int stack_pointer = 0;

            node_stack[stack_pointer++] = _root;

            // Traversal works like this: while there are nodes left on the stack, pop the topmost one. If it is a leaf,
            // traversal & shorten the ray against the triangles in the leaf node. If the node is an internal node,
            // traversal the ray against its left & right child node bboxes, and push those child nodes that were hit,
            // ordered by hit distance, to ensure the closest node gets traversed first.
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
                    const Node *left = &_nodes[node->leftFrom];
                    const Node *right = left + 1;

                    // transform ray to obb space for both left and right node
                    Ray leftRay = ray;
                    glm::vec4 rayOriginalLocal = (left->obb.invMatrix) * glm::vec4(leftRay.o, 1.0f);
                    glm::vec4 rayDirectionLocal = (left->obb.invMatrix) * glm::vec4(leftRay.d, 0.0f);
                    leftRay.o = glm::vec3(rayOriginalLocal.x, rayOriginalLocal.y, rayOriginalLocal.z);
                    leftRay.rd = glm::vec3(1.0f / rayDirectionLocal.x, 1.0f / rayDirectionLocal.y,
                                           1.0f / rayDirectionLocal.z);

                    Ray rightRay = ray;
                    rayOriginalLocal = (right->obb.invMatrix) * glm::vec4(rightRay.o, 1.0f);
                    rayDirectionLocal = (right->obb.invMatrix) * glm::vec4(rightRay.d, 0.0f);
                    rightRay.o = glm::vec3(rayOriginalLocal.x, rayOriginalLocal.y, rayOriginalLocal.z);
                    rightRay.rd = glm::vec3(1.0f / rayDirectionLocal.x, 1.0f / rayDirectionLocal.y,
                                            1.0f / rayDirectionLocal.z);

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

    bool ObbTree::traversal4x4(Ray4x4 &rays, const int maxIntersections) const
    {
        return false;
    }

    void ObbTree::construtBVH(const std::vector<Triangle> &triangles)
    {
        std::iota(_triangleIds.begin(), _triangleIds.end(), 0);

        std::transform(triangles.begin(), triangles.end(), _triangleCentroids.begin(), [](const Triangle &t)
        {
            return (t.vertices[0] + t.vertices[1] + t.vertices[2]) / 3.0f;
        });

        _nodes.reserve(triangles.size() * 2 - 1);
        _root = &_nodes.emplace_back(0, triangles.size());

        // construct obb in this function
        splitNode(_root);

        _tempMaxDepth = calculateMaxDepth(0);

        std::cerr << "NODE STRUCT SIZE: " << sizeof(Node) << std::endl;
        std::cerr << "BVH SIZE: " << _nodes.size() << std::endl;
        std::cerr << "BVH MAX DEPTH: " << _tempMaxDepth << std::endl;
        std::cerr << "TRIANGLE SIZE: " << _triangles.size() << std::endl;

        // Re-order triangles such that triangles for each node are adjacent in memory again. This should improve
        // data locality and avoids having to use indirection when iterating triangles for intersection
        linearize();
    }

    Node *ObbTree::splitNode(Node *const node)
    {
        // generate obb, with base class
        computeOBB(node);

        if (node->count > LEAF_SIZE)
        {
            auto split_plane = splitPlaneOBB(node, 32);

            if (split_plane)
            {
                auto split_index = partition(node->leftFrom, node->count, *split_plane);

                if (split_index)
                {
                    int left_index = _nodes.size();
                    _nodes.emplace_back(node->leftFrom, *split_index - node->leftFrom);
                    _nodes.emplace_back(*split_index, node->leftFrom + node->count - *split_index);

                    splitNode(&_nodes[left_index]);
                    splitNode(&_nodes[left_index + 1]);

                    // Update the original node to no longer directly contain triangles
                    node->leftFrom = left_index;
                    node->count = 0; // This node is now an internal node
                }
            }
        }

        return node;
    }

    std::optional<int> ObbTree::partition(const int from, const int count, const Plane &splitPlane)
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

    std::optional<Plane> ObbTree::splitPlaneOBB(const Node *const node, int maxSplitsPerDimension) const
    {
        const DiTO::OBB &obb = node->obb;
        std::optional<Plane> best_plane;

        std::array<glm::dvec3, 3> axes = {obb.v0, obb.v1, obb.v2};

        // Iterate over each axis of the OBB
        for (int axis = 0; axis < 3; ++axis)
        {
            float minExtent = std::numeric_limits<float>::infinity();
            float maxExtent = -std::numeric_limits<float>::infinity();

            // Find min and max extents of the triangles along the current axis
            for (int i = node->leftFrom; i < node->leftFrom + node->count; ++i)
            {
                const glm::dvec3 centroid = getCentroid(i);
                float projection = glm::dot(centroid - obb.mid, axes[axis]);

                minExtent = std::min(minExtent, projection);
                maxExtent = std::max(maxExtent, projection);
            }

            // Calculate the midpoint along the current axis
            double midPoint = (minExtent + maxExtent) * 0.5f;

            glm::dvec3 planePoint = obb.mid + axes[axis] * midPoint;
            glm::dvec3 planeNormal = axes[axis];

            best_plane = Plane(planePoint, planeNormal);
            break; // Break after the first axis for simplicity, or remove to select the 'best' axis
        }

        return best_plane;
    }

    void ObbTree::linearize()
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

    void ObbTree::computeOBBPerObj(const std::vector<Triangle> &object, Node *node)
    {
        std::vector<glm::dvec3> vertices;
        for (const auto &triangle: object)
        {
            for (const glm::vec3 &v: triangle.vertices)
            {
                vertices.push_back(v);
            }
        }

        // Compute the OBB using the DiTO algorithm, if there are vertices present
        if (!vertices.empty())
        {
            DiTO::DiTO_14(vertices.data(), vertices.size(), node->obb);
        }

        node->obb.ext = glm::max(node->obb.ext, glm::dvec3(0.001));

        // create obb matrix, transform unit AABB (-0.5 - 0.5) to obb space
        // Scale matrix
        glm::mat4 scaleMatrix = glm::scale(glm::mat4(1.0f), 2.0f * (glm::vec3(node->obb.ext)));

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

    void ObbTree::computeOBB(Node *node)
    {
        std::vector<glm::dvec3> vertices;
        for (int i = node->leftFrom; i < (node->leftFrom + node->count); ++i)
        {
            for (const glm::vec3 &v: getTriangle(i).vertices)
            {
                vertices.push_back(v);
            }
        }

        // Compute the OBB using the DiTO algorithm, if there are vertices present
        if (!vertices.empty())
        {
            DiTO::DiTO_14(vertices.data(), vertices.size(), node->obb);
        }

        node->obb.ext = glm::max(node->obb.ext, glm::dvec3(0.001));

        // create obb matrix, transform unit AABB (-0.5 - 0.5) to obb space
        // Scale matrix
        glm::mat4 scaleMatrix = glm::scale(glm::mat4(1.0f), 2.0f * (glm::vec3(node->obb.ext)));

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

    int ObbTree::calculateMaxDepth(int index, int currentDepth)
    {
        if (index >= _nodes.size() || _nodes[index].isLeaf()) return currentDepth;

        // Assuming right child immediately follows left child in the nodes vector
        int leftDepth = calculateMaxDepth(_nodes[index].leftFrom, currentDepth + 1);
        int rightDepth = calculateMaxDepth(_nodes[index].leftFrom + 1, currentDepth + 1);

        return std::max(leftDepth, rightDepth);
    }


}