#include "obbTree.h"

namespace core
{

    ObbTree::ObbTree(const std::vector<std::vector<Triangle>> &objects) :
            _nodeCount(0),
            _unitAABB(glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(0.5f, 0.5f, 0.5f))
    {
        _nodes.emplace_back(0, 0);
        for (const auto &object: objects)
        {
            _triangles.insert(_triangles.end(), object.begin(), object.end());
            init(object, ++_nodeCount);
            computeOBB(object, &_nodes[_nodeCount - 1]);
        }
        _root = &_nodes[0];
    }

    void ObbTree::init(const std::vector<Triangle> &triangles, unsigned int nodeIndex)
    {
        _nodes.emplace_back(nodeIndex, triangles.size());
    }

    void ObbTree::computeOBB(const std::vector<Triangle> &object, Node *node)
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

    bool ObbTree::intersect(Ray &ray, const int maxIntersections) const
    {
        const Node *node_stack[_nodes.size()];

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
}