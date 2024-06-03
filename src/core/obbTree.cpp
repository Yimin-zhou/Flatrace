#include <numeric>
#include <iostream>
#include <chrono>
#include <random>
#include <algorithm>
#include <mlpack/methods/kmeans/kmeans.hpp>
#include <mlpack/core.hpp>

#include "obbTree.h"
#include "src/utils/globalState.h"
#include "Tracy.hpp"

core::obb::ObbTree::ObbTree(const std::vector<Triangle> &triangles)
        :
        _failed(false),
        _triangles(triangles),
        _triangleIds(triangles.size()),
        _triangleCentroids(triangles.size()),
        _unitAABB(glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(0.5f, 0.5f, 0.5f)),
        m_nGroup(100),
        m_isTransformed(m_nGroup, false),
        m_clusterRayDirs(m_nGroup),
        m_clusterOBBs(m_nGroup)
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

    // Random grouping
//    m_transformationCache = std::vector<glm::mat4x4>(m_nGroup);
//    m_clusteredNodes = groupRandomNodes(m_nGroup);
//    cacheTransformations();

    // Kmeans
    m_transformationCache = std::vector<glm::mat4x4>(m_nGroup);
    m_clusteredNodes = clusterOBBs(m_nGroup);
    cacheTransformations();
}

bool core::obb::ObbTree::traversal(core::Ray &ray, const int maxIntersections, bool useClustering)
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
            ray.bvh_nodes_visited++;
            const core::obb::Node *const node = node_stack[--stack_pointer];

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
                intersectInternalNodes(left, right, ray, t_left, t_right, useClustering);

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

    if (node->count > LEAF_SIZE)
    {
        auto split_plane = splitPlaneMid(node, 32);

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
core::obb::ObbTree::splitPlaneMid(const Node *const node, int maxSplitsPerDimension) const
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

// Utility function to calculate the surface area of an OBB
template<typename F>
float surfaceArea(const DiTO::OBB<F>& obb) {
    // Approximate surface area using the extents
    return 2.0f * (obb.ext.x * obb.ext.y + obb.ext.y * obb.ext.z + obb.ext.z * obb.ext.x);
}

std::optional<core::Plane> core::obb::ObbTree::splitPlaneSAH(const Node* const node, int maxSplitsPerDimension) const {
    const float inf = std::numeric_limits<float>::infinity();
    float bestCost = inf;
    std::optional<Plane> bestPlane;

    // Check along each principal axis (v0, v1, v2)
    const glm::vec3 axes[3] = {
            glm::vec3(node->obb.v0.x, node->obb.v0.y, node->obb.v0.z),
            glm::vec3(node->obb.v1.x, node->obb.v1.y, node->obb.v1.z),
            glm::vec3(node->obb.v2.x, node->obb.v2.y, node->obb.v2.z)
    };

    for (const auto& axis : axes) {
        // Project centroids onto the axis
        std::vector<float> projections(node->count);
        for (int i = 0; i < node->count; ++i) {
            int index = node->leftFrom + i;
            const glm::vec3& centroid = _triangleCentroids[_triangleIds[index]];
            projections[i] = glm::dot(centroid, axis);
        }

        // Sort projections to find candidate split positions
        std::sort(projections.begin(), projections.end());

        // Evaluate each split plane
        for (int i = 1; i < maxSplitsPerDimension; ++i) {
            float splitValue = projections[(i * projections.size()) / maxSplitsPerDimension];

            // Calculate SAH cost
            DiTO::OBB<float> leftOBB, rightOBB;
            int leftCount = 0, rightCount = 0;
            for (int j = 0; j < node->count; ++j) {
                int index = node->leftFrom + j;
                const glm::vec3& centroid = _triangleCentroids[_triangleIds[index]];

                if (glm::dot(centroid, axis) <= splitValue) {
                    leftCount++;
                    // Update left OBB (not implemented here for simplicity)
                } else {
                    rightCount++;
                    // Update right OBB (not implemented here for simplicity)
                }
            }

            // Calculate surface areas of the resulting child OBBs
            float leftArea = surfaceArea(leftOBB);
            float rightArea = surfaceArea(rightOBB);

            // SAH cost: Area-weighted sum of child costs
            float sahCost = leftCount * leftArea + rightCount * rightArea;

            if (sahCost < bestCost) {
                bestCost = sahCost;
//                bestPlane = core::Plane{axis, splitValue};
            }
        }
    }

    return bestPlane;
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
//    for (int i = 0; i < numOBBs; ++i)
//    {
//        DiTO::OBB<float> obb;
//        obb.mid = DiTO::Vector<float>(0.0f, 0.0f, 0.0f);
//        obb.v0 = DiTO::Vector<float>(1.0f, 0.0f, 0.0f);
//        obb.v1 = DiTO::Vector<float>(0.0f, 1.0f, 0.0f);
//        obb.v2 = DiTO::Vector<float>(0.0f, 0.0f, 1.0f);
//        obb.ext = DiTO::Vector<float>(0.5f, 0.5f, 0.5f);
//        m_preGeneratedOBBs.push_back(obb);
//    }
}

std::vector<std::vector<core::obb::Node>> core::obb::ObbTree::clusterOBBs(int num_clusters)
{
    std::vector<std::vector<core::obb::Node>> finalGroups(num_clusters);
    std::vector<core::obb::Node> leafNodes;

    // Extract leaf nodes
    for (auto& node : _nodes)
    {
        if (node.isLeaf())
        {
            leafNodes.push_back(node);
        }
    }

    // Flatten each OBB instance
    std::vector<Eigen::Matrix<float, Eigen::Dynamic, 1>> data;
    for (const auto& node : leafNodes)
    {
        data.push_back(flatten(node.obb));
    }

    // Convert Eigen matrices to an Armadillo matrix
    size_t num_samples = data.size();
    size_t dim = data[0].size();
    arma::mat dataset(dim, num_samples);

    for (size_t i = 0; i < num_samples; ++i)
    {
        for (size_t j = 0; j < dim; ++j)
        {
            dataset(j, i) = data[i](j);
        }
    }

    // Perform K-means clustering
    arma::Row<size_t> assignments;
    mlpack::kmeans::KMeans<> kmeans;
    kmeans.Cluster(dataset, num_clusters, assignments);

    // Create clusters
    std::vector<std::vector<core::obb::Node>> clusters(num_clusters);
    for (size_t i = 0; i < assignments.n_elem; ++i)
    {
        clusters[assignments[i]].push_back(leafNodes[i]);
    }

    std::cerr << "Clustered " << leafNodes.size() << " OBBs into " << num_clusters << " clusters." << std::endl;

    return clusters;
}

void core::obb::ObbTree::cacheTransformations()
{
    // set one transformation matrix for one group, and store in m_cachedTransformation
    for (auto& group : m_clusteredNodes)
    {
        if (group.empty())
        {
            continue;
        }

        // move all triangles to one point (base on centroid) and calculate the obb for the group
        DiTO::OBB<float> groupOBB;
        glm::vec3 groupCentroid(0.0f);
        size_t totalTriangles = 0;

        for (const auto& node : group)
        {
            for (int i = node.leftFrom; i < (node.leftFrom + node.count); ++i)
            {
                groupCentroid += getCentroid(i);
                totalTriangles++;
            }
        }
        // calculate the centroid of the group
        groupCentroid /= totalTriangles;

        std::vector<DiTO::Vector<float>> vertices;

        for (const auto& node : group)
        {
            for (int i = node.leftFrom; i < (node.leftFrom + node.count); ++i)
            {
                for (const auto &v: getTriangle(i).vertices)
                {
                    DiTO::Vector<float> vertex(v.x - groupCentroid.x, v.y - groupCentroid.y, v.z - groupCentroid.z);
                    vertices.push_back(vertex);
                }
            }
        }

        // Compute the OBB using the DiTO algorithm, if there are vertices present
        if (!vertices.empty())
        {
            DiTO::DiTO_14(vertices.data(), vertices.size(), groupOBB);
        }
        m_transformationCache[group[0].groupNumber] = groupOBB.invMatrix;
        // For obb visualization
//        m_clusterOBBs[group[0].groupNumber] = groupOBB;
    }
    std::cout << "Cached transformations" << std::endl;
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

void core::obb::ObbTree::triangleIntersection(const core::obb::Node *const node, core::Ray &ray)
{
    for (int j = node->leftFrom; j < (node->leftFrom + node->count); j++)
    {
    core::intersect(_triangles[j], ray);
    }
}

void core::obb::ObbTree::intersectInternalNodes(const Node *left, const Node *right,
                                                core::Ray &ray, float& outLeft, float& outRight,  bool useClustering)
{
    Ray leftRay = ray;
    Ray rightRay = ray;

    // Use grouped transformations for leaf nodes
    if (left->isLeaf() && useClustering)
    {
        // Calculate the ray in the local space of the OBB (for each cluster)
        if (!m_isTransformed[left->groupNumber])
        {
            m_clusterRayDirs[left->groupNumber] = m_transformationCache[left->groupNumber] * glm::vec4(leftRay.d, 0.0f);
            m_isTransformed[left->groupNumber] = true;
        }

        glm::vec4 rayOriginalLocal = m_transformationCache[left->groupNumber] * glm::vec4(leftRay.o, 1.0f);
        glm::vec3 rayDirectionLocal = m_clusterRayDirs[left->groupNumber];
        leftRay.o = glm::vec3(rayOriginalLocal.x, rayOriginalLocal.y, rayOriginalLocal.z);
        leftRay.rd = glm::vec3(1.0f / rayDirectionLocal.x, 1.0f / rayDirectionLocal.y, 1.0f / rayDirectionLocal.z);
    }
    else if (right->isLeaf() && useClustering)
    {
        if (!m_isTransformed[right->groupNumber])
        {
            m_clusterRayDirs[right->groupNumber] = m_transformationCache[right->groupNumber] * glm::vec4(leftRay.d, 0.0f);
            m_isTransformed[right->groupNumber] = true;
        }

        glm::vec4 rayOriginalLocal = m_transformationCache[left->groupNumber] * glm::vec4(leftRay.o, 1.0f);
        glm::vec3 rayDirectionLocal = m_clusterRayDirs[right->groupNumber];
        leftRay.o = glm::vec3(rayOriginalLocal.x, rayOriginalLocal.y, rayOriginalLocal.z);
        leftRay.rd = glm::vec3(1.0f / rayDirectionLocal.x, 1.0f / rayDirectionLocal.y, 1.0f / rayDirectionLocal.z);
    }
    else
    {
        glm::vec4 rayOriginalLocal = (left->obb.invMatrix) * glm::vec4(leftRay.o, 1.0f);
        glm::vec4 rayDirectionLocal = (left->obb.invMatrix) * glm::vec4(leftRay.d, 0.0f);
        leftRay.o = glm::vec3(rayOriginalLocal.x, rayOriginalLocal.y, rayOriginalLocal.z);
        leftRay.rd = glm::vec3(1.0f / rayDirectionLocal.x, 1.0f / rayDirectionLocal.y, 1.0f / rayDirectionLocal.z);

        rayOriginalLocal = (right->obb.invMatrix) * glm::vec4(rightRay.o, 1.0f);
        rayDirectionLocal = (right->obb.invMatrix) * glm::vec4(rightRay.d, 0.0f);
        rightRay.o = glm::vec3(rayOriginalLocal.x, rayOriginalLocal.y, rayOriginalLocal.z);
        rightRay.rd = glm::vec3(1.0f / rayDirectionLocal.x, 1.0f / rayDirectionLocal.y, 1.0f / rayDirectionLocal.z);
    }

    outLeft = core::intersectAABB(_unitAABB, leftRay);
    outRight = core::intersectAABB(_unitAABB, rightRay);
}

