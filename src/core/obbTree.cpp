#include <numeric>
#include "obbTree.h"
#include "src/utils/globalState.h"
#include "Tracy.hpp"

core::obb::ObbTree::ObbTree(const std::vector<Triangle> &triangles, const glm::vec3 &rayDir)
        :
        _failed(false),
        _triangles(triangles),
        _triangleIds(triangles.size()),
        _triangleCentroids(triangles.size()),
        _unitAABB(glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(0.5f, 0.5f, 0.5f)),
        m_rayDir(rayDir)
{
    std::iota(_triangleIds.begin(), _triangleIds.end(), 0);

    std::transform(triangles.begin(), triangles.end(), _triangleCentroids.begin(), [](const Triangle &t)
    {
        return (t.vertices[0] + t.vertices[1] + t.vertices[2]) / 3.0f;
    });

    _nodes.reserve(triangles.size() * 2 - 1);
    _root = &_nodes.emplace_back(0, triangles.size());

    splitNode(_root);

    // BVH stats
    _maxDepth = calculateMaxLeafDepth(_root);
    int minDepth = calculateMinLeafDepth(_root);
    collectLeafDepths(_root);

    std::cerr << "BVH NODE AMOUNT: " << _nodes.size() << std::endl;
    std::cerr << "BVH MAX DEPTH: " << _maxDepth << std::endl;
    std::cerr << "BVH MIN DEPTH: " << minDepth << std::endl;

    // Re-order triangles such that triangles for each node are adjacent in memory again. This should improve
    // data locality and avoids having to use indirection when iterating triangles for intersection
    linearize();
}

bool core::obb::ObbTree::traversal(core::Ray &ray, const int maxIntersections) const
{
    ZoneScopedN("OBB Tree Traversal");

    const Node *node_stack[_nodes.size()];

//    if (core::intersectAABB(_root->bbox, ray) == INF)
//    {
//        return false;
//    }

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

                const Node *left = &_nodes[node->leftFrom];
                const Node *right = left + 1;

                // transform ray to obb space for both left and right node
//                Ray leftRay = ray;
//                glm::vec4 rayOriginalLocal = (left->obb.invMatrix) * glm::vec4(leftRay.o, 1.0f);
//                glm::vec4 rayDirectionLocal = (left->obb.invMatrix) * glm::vec4(leftRay.d, 0.0f);
//                leftRay.o = glm::vec3(rayOriginalLocal.x, rayOriginalLocal.y, rayOriginalLocal.z);
//                leftRay.rd = glm::vec3(1.0f / rayDirectionLocal.x, 1.0f / rayDirectionLocal.y,
//                                       1.0f / rayDirectionLocal.z);
//
//                Ray rightRay = ray;
//                rayOriginalLocal = (right->obb.invMatrix) * glm::vec4(rightRay.o, 1.0f);
//                rayDirectionLocal = (right->obb.invMatrix) * glm::vec4(rightRay.d, 0.0f);
//                rightRay.o = glm::vec3(rayOriginalLocal.x, rayOriginalLocal.y, rayOriginalLocal.z);
//                rightRay.rd = glm::vec3(1.0f / rayDirectionLocal.x, 1.0f / rayDirectionLocal.y,
//                                        1.0f / rayDirectionLocal.z);
//
                Ray leftRay = ray;
                glm::vec3 rayOriginalLocal = glm::vec3(left->obb.invMatrix * glm::vec4(leftRay.o, 1.0f));
                leftRay.o = rayOriginalLocal;
                leftRay.rd = glm::vec3(1.0f/ left->transformedRayDir.x, 1.0f/ left->transformedRayDir.y, 1.0f/ left->transformedRayDir.z);

                Ray rightRay = ray;
                rayOriginalLocal = glm::vec3(right->obb.invMatrix * glm::vec4(rightRay.o, 1.0f));
                rightRay.o = rayOriginalLocal;
                rightRay.rd = glm::vec3(1.0f/ right->transformedRayDir.x, 1.0f/ right->transformedRayDir.y, 1.0f/ right->transformedRayDir.z);

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

bool core::obb::ObbTree::traversal4x4(core::Ray4x4 &rays, const int maxIntersections) const
{
    return false;
}

core::obb::Node *core::obb::ObbTree::splitNode(core::obb::Node *const node)
{
    computeOBB<float>(node);
    node->transformedRayDir = glm::vec3(node->obb.invMatrix * glm::vec4(m_rayDir, 0.0f));

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

std::optional<core::Plane>
core::obb::ObbTree::splitPlaneOBB(const core::obb::Node *const node, int maxSplitsPerDimension) const
{
    const DiTO::OBB obb = node->obb;
    std::optional<Plane> best_plane;

    std::array<glm::vec3, 3> axes = {glm::vec3(obb.v0.x, obb.v0.y, obb.v0.z),
                                     glm::vec3(obb.v1.x, obb.v1.y, obb.v1.z),
                                     glm::vec3(obb.v2.x, obb.v2.y, obb.v2.z)};

    // Iterate over each axis of the OBB
    for (int axis = 0; axis < 3; ++axis)
    {
        float minExtent = std::numeric_limits<float>::infinity();
        float maxExtent = -std::numeric_limits<float>::infinity();

        // Find min and max extents of the triangles along the current axis
        for (int i = node->leftFrom; i < node->leftFrom + node->count; ++i)
        {
            const glm::vec3 centroid = getCentroid(i);
            float projection = glm::dot(centroid - glm::vec3(obb.mid.x, obb.mid.y, obb.mid.z),
                                        axes[axis]);

            minExtent = std::min(minExtent, projection);
            maxExtent = std::max(maxExtent, projection);
        }

        // Calculate the midpoint along the current axis
        float midPoint = (minExtent + maxExtent) * 0.5f;

        glm::dvec3 planePoint = glm::vec3(obb.mid.x, obb.mid.y, obb.mid.z)+ axes[axis] * midPoint;
        glm::dvec3 planeNormal = axes[axis];

        best_plane = Plane(planePoint, planeNormal);
        break; // Break after the first axis for simplicity, or remove to select the 'best' axis
    }

    return best_plane;
}

std::optional<int> core::obb::ObbTree::partition(const int from, const int count, const core::Plane &splitPlane)
{
    int left_to = from;
    int right_from = from + count;

    while (left_to < right_from)
    {
        const glm::vec3 c = getCentroid(left_to);

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


void core::obb::ObbTree::linearize()
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

void core::obb::ObbTree::preGenerateOBBs(int numOBBs)
{
    for (int i = 0; i < numOBBs; ++i)
    {
        DiTO::OBB<float> obb;
        obb.mid = DiTO::Vector<float>(0.0f, 0.0f, 0.0f);
        obb.v0 = DiTO::Vector<float>(1.0f, 0.0f, 0.0f);
        obb.v1 = DiTO::Vector<float>(0.0f, 1.0f, 0.0f);
        obb.v2 = DiTO::Vector<float>(0.0f, 0.0f, 1.0f);
        obb.ext = DiTO::Vector<float>(0.5f, 0.5f, 0.5f);
        m_preGeneratedOBBs.push_back(obb);
    }
}

std::vector<std::vector<core::obb::Node>> core::obb::ObbTree::groupSimilarOBBs(float similarityThreshold)
{
    std::vector<std::vector<Node>> groups;
    std::unordered_map<int, std::vector<Node>> similarityGroups;

    for (const auto& node : _nodes)
    {
        bool foundGroup = false;
        for (auto& [key, group] : similarityGroups)
        {
            if (isSimilar(node.obb, m_preGeneratedOBBs[key], similarityThreshold))
            {
                group.push_back(node);
                foundGroup = true;
                break;
            }
        }

        if (!foundGroup)
        {
            int newKey = m_preGeneratedOBBs.size();
            m_preGeneratedOBBs.push_back(node.obb);
            similarityGroups[newKey].push_back(node);
        }
    }

    for (auto& [key, group] : similarityGroups)
    {
        groups.push_back(std::move(group));
    }

    return groups;
}

bool
core::obb::ObbTree::isSimilar(const DiTO::OBB<float> &obb1, const DiTO::OBB<float> &obb2, float similarityThreshold)
{
    // use dot product to calculate the angle between the two OBBs
    float dotV0 = glm::dot(glm::vec3(obb1.v0.x, obb1.v0.y, obb1.v0.z), glm::vec3(obb2.v0.x, obb2.v0.y, obb2.v0.z));
    float dotV1 = glm::dot(glm::vec3(obb1.v1.x, obb1.v1.y, obb1.v1.z), glm::vec3(obb2.v1.x, obb2.v1.y, obb2.v1.z));
    float dotV2 = glm::dot(glm::vec3(obb1.v2.x, obb1.v2.y, obb1.v2.z), glm::vec3(obb2.v2.x, obb2.v2.y, obb2.v2.z));

    // check if the angle between the two OBBs is less than the threshold
    return (dotV0 > similarityThreshold && dotV1 > similarityThreshold && dotV2 > similarityThreshold);
}

void core::obb::ObbTree::cacheTransformations()
{
//    for (const auto& node : _nodes)
//    {
//        // Compute the transformation matrix for the OBB
//        glm::mat4 transformation = glm::translate(glm::mat4(1.0f), node.obb.center) *
//                                   glm::mat4(node.obb.orientation) *
//                                   glm::scale(glm::mat4(1.0f), node.obb.extents);
//
//        m_transformationCache[node.leftFrom] = transformation;
//    }
}

int core::obb::ObbTree::calculateMaxLeafDepth(const core::obb::Node* node, int depth) const
{
    if (node->isLeaf())
    {
        return depth;
    }

    int leftDepth = calculateMaxLeafDepth(&_nodes[node->leftFrom], depth + 1);
    int rightDepth = calculateMaxLeafDepth(&_nodes[node->leftFrom + 1], depth + 1);

    return std::max(leftDepth, rightDepth);
}

int core::obb::ObbTree::calculateMinLeafDepth(const core::obb::Node *node, int depth) const
{
    if (node->isLeaf())
    {
        return depth;
    }

    int leftDepth = calculateMinLeafDepth(&_nodes[node->leftFrom], depth + 1);
    int rightDepth = calculateMinLeafDepth(&_nodes[node->leftFrom + 1], depth + 1);

    return std::min(leftDepth, rightDepth);
}

void core::obb::ObbTree::collectLeafDepths(const core::obb::Node *node, int currentDepth)
{
    if (node->isLeaf())
    {
        m_leafDepths.push_back(currentDepth);
        return;
    }

    collectLeafDepths(&_nodes[node->leftFrom], currentDepth + 1);
    collectLeafDepths(&_nodes[node->leftFrom + 1], currentDepth + 1);
}

template<typename F>
void core::obb::ObbTree::computeOBB(core::obb::Node *node)
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
