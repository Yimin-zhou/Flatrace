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
#include "intersect.h"

#ifdef IS_X86

#include <immintrin.h>

#else
#include <simde/x86/avx2.h>
#endif

core::obb::ObbTree::ObbTree(const std::vector<Triangle> &triangles, bool useSAH, bool useClustering, int binSize,
                            int num_clusters, bool useMedian)
        :
        m_failed(false),
        m_triangles(triangles),
        m_triangleIds(triangles.size()),
        m_triangleCentroids(triangles.size()),
        m_unitAABB(glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(0.5f, 0.5f, 0.5f)),
        m_nGroup(num_clusters),
        m_useClustering(useClustering),
        m_clusterOBBs(m_nGroup),
        m_binSize(binSize),
        m_useMedian(useMedian),
        m_useSAH(useSAH),
        m_cachedClusterRaydirs(m_nGroup)
{
    std::iota(m_triangleIds.begin(), m_triangleIds.end(), 0);

    std::transform(triangles.begin(), triangles.end(), m_triangleCentroids.begin(), [](const Triangle &t)
    {
        return (t.vertices[0] + t.vertices[1] + t.vertices[2]) / 3.0f;
    });

    m_nodes.reserve(triangles.size() * 2 - 1);
    m_root = &m_nodes.emplace_back(0, triangles.size(), 0);

    splitNode(m_root);

    // BVH stats
    m_maxDepth = calculateMaxLeafDepth(m_root);
    int minDepth = calculateMinLeafDepth(m_root);
    collectLeafDepths(m_root);

//    std::cerr << "BVH NODE AMOUNT: " << _nodes.size() << std::endl;
//    std::cerr << "BVH MAX DEPTH: " << _maxDepth << std::endl;
//    std::cerr << "BVH MIN DEPTH: " << minDepth << std::endl;

    // Re-order triangles such that triangles for each node are adjacent in memory again. This should improve
    // data locality and avoids having to use indirection when iterating triangles for intersection
    linearize();

    if (m_useClustering)
    {
        // Kmeans
        m_clusteredNodes = clusterOBBsKmeans(m_nGroup);
        cacheTransformations();
    }
}

bool core::obb::ObbTree::traversal(core::Ray &ray, const int maxIntersections)
{
    ZoneScopedN("OBB Tree Traversal");

    const Node *node_stack[m_nodes.size()];

//    if (core::intersectAABB(m_root->bbox, ray) == INF)
//    {
//        return false;
//    }

    for (int i = 0; i < maxIntersections; i++)
    {
        int stack_pointer = 0;

        node_stack[stack_pointer++] = m_root;

        while (stack_pointer != 0)
        {
            ray.bvh_nodes_visited++;
            const core::obb::Node *const node = node_stack[--stack_pointer];

            if (node->isLeaf())
            {
                triangleIntersection(node, ray);
            } else
            {
                const Node *left = &m_nodes[node->leftFrom];
                const Node *right = left + 1;
                float t_left = 0;
                float t_right = 0;
                intersectInternalNodes(left, ray, t_left);
                intersectInternalNodes(right, ray, t_right);

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

bool core::obb::ObbTree::traversal4x4(core::Ray4x4 &rays, const int maxIntersections, const std::vector<glm::vec3> &cachedClusterRaydirs) const
{
    static const __m256 inf_x8 = _mm256_set1_ps(INF);

    const Node *node_stack[m_nodes.size()];

    bool hit = false;
    bool dead = false;

    for (int i = 0; !dead && (i < maxIntersections); i++)
    {
        int stack_pointer = 0;

        node_stack[stack_pointer++] = m_root;

        while (stack_pointer != 0)
        {
            const Node *const node = node_stack[--stack_pointer];

            if (node->isLeaf())
            {
                for (int i = node->leftFrom; i < (node->leftFrom + node->count); i++)
                {
                    core::intersect4x4(m_triangles[i], rays); // Triangle intersection
                }
            } else
            {
                const Node *child_0 = &m_nodes[node->leftFrom];
                const Node *child_1 = child_0 + 1;
                float t_left = 0;
                float t_right = 0;

                t_left = intersectInternalNodes4x4(child_0, rays, cachedClusterRaydirs);
                t_right = intersectInternalNodes4x4(child_1, rays, cachedClusterRaydirs);

                if (t_left > t_right)
                {
                    std::swap(t_left, t_right);
                    std::swap(child_0, child_1);
                }

                if (t_left != INF)
                {
                    if (t_right != INF)
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

core::obb::Node *core::obb::ObbTree::splitNode(core::obb::Node *const node)
{
    computeOBB<float>(node);

    if (node->index == 0)
    {
        for (int i = node->leftFrom; i < (node->leftFrom + node->count); i++)
        {
            for (const glm::vec3 &v: getTriangle(i).vertices)
            {
                node->bbox.min = glm::min(node->bbox.min, v);
                node->bbox.max = glm::max(node->bbox.max, v);
            }
        }
    }

    if (node->count > TracerState::LEAF_SIZE)
    {
        std::optional<Plane> split_plane;
        if (m_useSAH)
        {
            split_plane = splitPlaneSAH(node, node->leftFrom, node->count);
        } else
        {
            split_plane = splitPlaneMid(node);
        }

        if (split_plane)
        {
            auto split_index = partition(node->leftFrom, node->count, *split_plane);

            if (split_index)
            {
                int left_index = m_nodes.size();
                m_nodes.emplace_back(node->leftFrom, *split_index - node->leftFrom, left_index);
                m_nodes.emplace_back(*split_index, node->leftFrom + node->count - *split_index, left_index + 1);

                splitNode(&m_nodes[left_index]);
                splitNode(&m_nodes[left_index + 1]);

                // Update the original node to no longer directly contain triangles
                node->leftFrom = left_index;
                node->count = 0; // This node is now an internal node
            }
        }
    }

    return node;
}

std::optional<core::Plane>
core::obb::ObbTree::splitPlaneMid(const Node *const node) const
{
    const DiTO::OBB obb = node->obb;
    std::optional<Plane> best_plane;

    std::vector<glm::vec3> axes = {glm::vec3(obb.v0.x, obb.v0.y, obb.v0.z),
                                   glm::vec3(obb.v1.x, obb.v1.y, obb.v1.z),
                                   glm::vec3(obb.v2.x, obb.v2.y, obb.v2.z)
    };

    // Find the axis with the max length
    int axis = 0;
    float max_length = glm::length(axes[0]);
    for (int i = 1; i < 3; i++)
    {
        float length = glm::length(axes[i]);
        if (length > max_length)
        {
            max_length = length;
            axis = i;
        }
    }

    float minExtent = std::numeric_limits<float>::infinity();
    float maxExtent = -std::numeric_limits<float>::infinity();

    // Median method
    std::vector<float> projections;

    // Find min and max extents of the triangles along the current axis
    for (int i = node->leftFrom; i < node->leftFrom + node->count; ++i)
    {
        const glm::vec3 centroid = getCentroid(i);
        float projection = glm::dot(centroid - glm::vec3(obb.mid.x, obb.mid.y, obb.mid.z), axes[axis]);

        minExtent = std::min(minExtent, projection);
        maxExtent = std::max(maxExtent, projection);
        // Median method
        if (m_useMedian)
        {
            projections.push_back(projection);
        }
    }

    // Calculate the midpoint along the current axis
    float midPoint = (minExtent + maxExtent) * 0.5f;

    // Sort // Median method
    if (m_useMedian)
    {
        std::sort(projections.begin(), projections.end());
        midPoint = projections[projections.size() / 2];
    }


    glm::dvec3 planePoint = glm::vec3(obb.mid.x, obb.mid.y, obb.mid.z) + axes[axis] * midPoint;

    glm::dvec3 planeNormal = axes[axis];

    best_plane = Plane(planePoint, planeNormal);

    return best_plane;
}


float core::obb::ObbTree::evaluateSAH(const Node *const node, const glm::vec3 &axis, const float candidateProj) const
{
    // determine triangle counts and bounds for this split candidate
    DiTO::OBB<float> leftBox;
    DiTO::OBB<float> rightBox;
    glm::vec3 mid = glm::vec3(node->obb.mid.x, node->obb.mid.y, node->obb.mid.z);

    int leftCount = 0, rightCount = 0;

    std::vector<DiTO::Vector<float>> leftVertices;
    std::vector<DiTO::Vector<float>> rightVertices;

    for (uint i = 0; i < node->count; i++)
    {
        core::Triangle triangle = getTriangle(node->leftFrom + i);
        float centroidProj = glm::dot(getCentroid(node->leftFrom + i) - mid, axis);

        if (centroidProj < candidateProj)
        {
            for (int j = 0; j < 3; j++)
            {
                leftVertices.emplace_back(triangle.vertices[j].x, triangle.vertices[j].y, triangle.vertices[j].z);
            }
            leftCount++;
        } else
        {
            for (int j = 0; j < 3; j++)
            {
                rightVertices.emplace_back(triangle.vertices[j].x, triangle.vertices[j].y, triangle.vertices[j].z);
            }
            rightCount++;
        }
    }

    DiTO::DiTO_14(leftVertices.data(), leftVertices.size(), leftBox);
    DiTO::DiTO_14(rightVertices.data(), rightVertices.size(), rightBox);

    float leftArea = leftBox.area();
    float rightArea = rightBox.area();
    float cost = leftCount * leftArea + rightCount * rightArea;

    return cost > 0 ? cost : 1e30f;
}

std::optional<core::Plane>
core::obb::ObbTree::splitPlaneSAH(const Node *const node, const int from, const int count) const
{
    glm::vec3 bestAxis = glm::vec3(0.0f);
    float bestCost = std::numeric_limits<float>::infinity();
    glm::vec3 bestPos = glm::vec3(0.0f);

    float parentCost = node->obb.area() * count;

    const int splitsPerDimension = std::min(count, m_binSize);

    glm::vec3 mid = glm::vec3(node->obb.mid.x, node->obb.mid.y, node->obb.mid.z);
    glm::vec3 ext = glm::vec3(node->obb.ext.x, node->obb.ext.y, node->obb.ext.z);
    std::vector<glm::vec3> axes =
            {
                    glm::vec3(node->obb.v0.x, node->obb.v0.y, node->obb.v0.z),
                    glm::vec3(node->obb.v1.x, node->obb.v1.y, node->obb.v1.z),
                    glm::vec3(node->obb.v2.x, node->obb.v2.y, node->obb.v2.z)
            };

    for (int a = 0; a < 3; ++a)
    {
        std::vector<SplitBin> bins(splitsPerDimension);

        float boundsMinProj = 1e30f;
        float boundsMaxProj = -1e30f;
        for (int i = 0; i < node->count; i++)
        {
            glm::vec3 centroid = getCentroid(from + i);
            float proj = glm::dot(centroid - mid, axes[a]);
            boundsMinProj = std::min(boundsMinProj, proj);
            boundsMaxProj = std::max(boundsMaxProj, proj);
        }

        if (((boundsMaxProj - boundsMinProj) / bins.size()) <= 1e-6)
        {
            continue;
        }

        float scale = splitsPerDimension / (boundsMaxProj - boundsMinProj);

        for (int i = 0; i < node->count; i++)
        {
            core::Triangle triangle = getTriangle(from + i);
            int binIndex = std::min(
                    static_cast<int>((glm::dot(getCentroid(from + i) - mid, axes[a]) - boundsMinProj) * scale),
                    static_cast<int>(splitsPerDimension - 1));

            bins[binIndex].triangleCount++;
            bins[binIndex].obbBoundVertices.emplace_back(triangle.vertices[0].x, triangle.vertices[0].y,
                                                         triangle.vertices[0].z);
            bins[binIndex].obbBoundVertices.emplace_back(triangle.vertices[1].x, triangle.vertices[1].y,
                                                         triangle.vertices[1].z);
            bins[binIndex].obbBoundVertices.emplace_back(triangle.vertices[2].x, triangle.vertices[2].y,
                                                         triangle.vertices[2].z);
        }

        std::vector<float> leftArea(splitsPerDimension - 1);
        std::vector<float> rightArea(splitsPerDimension - 1);
        std::vector<int> leftCount(splitsPerDimension - 1);
        std::vector<int> rightCount(splitsPerDimension - 1);

        std::vector<DiTO::Vector<float>> leftBoxVertices, rightBoxVertices;
        int leftTrianglesCountSum = 0, rightTrianglesCountSum = 0;

        // Calculate the area of the left and right of each bin
        for (int i = 0; i < splitsPerDimension - 1; i++)
        {
            leftTrianglesCountSum += bins[i].triangleCount;
            leftCount[i] = leftTrianglesCountSum;
            leftBoxVertices.insert(leftBoxVertices.end(), bins[i].obbBoundVertices.begin(),
                                   bins[i].obbBoundVertices.end());
            DiTO::OBB<float> tempOBBLeft;
            DiTO::DiTO_14(leftBoxVertices.data(), leftBoxVertices.size(), tempOBBLeft);
            leftArea[i] = tempOBBLeft.area();

            rightTrianglesCountSum += bins[splitsPerDimension - 1 - i].triangleCount;
            rightCount[splitsPerDimension - 2 - i] = rightTrianglesCountSum;
            rightBoxVertices.insert(rightBoxVertices.end(), bins[splitsPerDimension - 1 - i].obbBoundVertices.begin(),
                                    bins[splitsPerDimension - 1 - i].obbBoundVertices.end());
            DiTO::OBB<float> tempOBBRight;
            DiTO::DiTO_14(rightBoxVertices.data(), rightBoxVertices.size(), tempOBBRight);
            rightArea[splitsPerDimension - 2 - i] = tempOBBRight.area();
        }

        scale = (boundsMaxProj - boundsMinProj) / splitsPerDimension;

        for (int i = 1; i < splitsPerDimension - 1; i++)
        {
            float planeCost = leftCount[i] * leftArea[i] + rightCount[i] * rightArea[i];
            if (planeCost < bestCost)
            {
                bestCost = planeCost;
                bestAxis = axes[a];
                bestPos = mid + axes[a] * (boundsMinProj + scale * (i + 1));
            }
        }
    }

    if (bestCost >= parentCost)
    {
        return std::nullopt;
    }

    return std::optional<Plane>(Plane(bestPos, bestAxis));

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
            std::swap(m_triangleIds[left_to], m_triangleIds[--right_from]);
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

    for (const int triangle_id: m_triangleIds)
    {
        linearized_triangle_ids.push_back(linearized_triangles.size());
        linearized_triangles.push_back(m_triangles[triangle_id]);
    }

    std::swap(m_triangleIds, linearized_triangle_ids);
    std::swap(m_triangles, linearized_triangles);
}

std::vector<std::vector<core::obb::Node>> core::obb::ObbTree::clusterOBBsKmeans(int num_clusters)
{
    std::vector<core::obb::Node> leafNodes;
    std::vector<size_t> leafNodeIndices;

    // Extract leaf nodes and their indices
    for (size_t i = 0; i < m_nodes.size(); ++i)
    {
        if (m_nodes[i].isLeaf())
        {
            leafNodes.push_back(m_nodes[i]);
            leafNodeIndices.push_back(i);
        }
    }
    m_leafSize = leafNodes.size();

    std::vector<Eigen::Matrix<float, Eigen::Dynamic, 1>> data;
    for (const auto &node: leafNodes)
    {
        data.push_back(DiTO::flatten(node.obb));
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

    // Create clusters and update the original nodes with new group numbers
    std::vector<std::vector<core::obb::Node>> clusters(num_clusters);
    for (size_t i = 0; i < assignments.n_elem; ++i)
    {
        leafNodes[i].groupNumber = assignments[i];
        m_nodes[leafNodeIndices[i]].groupNumber = assignments[i];
        clusters[assignments[i]].push_back(leafNodes[i]);
    }

    std::cerr << "Clustered " << leafNodes.size() << " OBBs into " << num_clusters << " clusters." << std::endl;

    return clusters;
}

void core::obb::ObbTree::cacheTransformations()
{
    m_transformationCache = std::vector<glm::mat4x4>(m_nGroup);

    for (auto& group : m_clusteredNodes)
    {
        if (group.empty())
        {
            continue;
        }

        // Calculate the group OBB based on the vertices of all nodes
        DiTO::OBB<float> groupOBB;
        std::vector<DiTO::Vector<float>> tempVertices;

        for (const auto& node : group)
        {
            for (int i = node.leftFrom; i < (node.leftFrom + node.count); ++i)
            {
                for (const auto &v: getTriangle(i).vertices)
                {
                    DiTO::Vector<float> vertex(v.x, v.y, v.z);
                    tempVertices.push_back(vertex);
                }
            }
        }

        // Calculate the OBB for the entire group of vertices
        DiTO::DiTO_14(tempVertices.data(), tempVertices.size(), groupOBB);
        glm::vec3 groupCenter = glm::vec3(groupOBB.mid.x, groupOBB.mid.y, groupOBB.mid.z);

        // Translate all vertices to the group center
        std::vector<DiTO::Vector<float>> vertices;
        for (const auto& node : group)
        {
            glm::vec3 translationVector = groupCenter - glm::vec3(node.obb.mid.x, node.obb.mid.y, node.obb.mid.z);
            for (int i = node.leftFrom; i < (node.leftFrom + node.count); ++i)
            {
                for (const auto& v : getTriangle(i).vertices)
                {
                    DiTO::Vector<float> vertex(v.x + translationVector.x, v.y + translationVector.y, v.z + translationVector.z);
                    vertices.push_back(vertex);
                }
            }
        }

        // Recompute the group OBB using the translated vertices if there are vertices present
        if (!vertices.empty())
        {
            DiTO::DiTO_14(vertices.data(), vertices.size(), groupOBB);
        }

        // Slightly expand the OBB extents
        groupOBB.ext = DiTO::Vector<float>(groupOBB.ext.x + 0.001f, groupOBB.ext.y + 0.001f, groupOBB.ext.z + 0.001f);

        // Create OBB matrix: transform unit AABB (-0.5 to 0.5) to OBB space
        glm::mat4 scaleMatrix = glm::scale(glm::mat4(1.0f), 2.0f * glm::vec3(groupOBB.ext.x, groupOBB.ext.y, groupOBB.ext.z));

        glm::mat4 rotationMatrix = glm::mat4(
                glm::vec4(glm::vec3(groupOBB.v0.x, groupOBB.v0.y, groupOBB.v0.z), 0.0f),
                glm::vec4(glm::vec3(groupOBB.v1.x, groupOBB.v1.y, groupOBB.v1.z), 0.0f),
                glm::vec4(glm::vec3(groupOBB.v2.x, groupOBB.v2.y, groupOBB.v2.z), 0.0f),
                glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)
        );

        glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(groupOBB.mid.x, groupOBB.mid.y, groupOBB.mid.z));

        // Compute the inverse matrix for the OBB
        groupOBB.invMatrix = glm::inverse(translationMatrix * rotationMatrix * scaleMatrix);

        // Cache the transformation matrix for the group
        m_transformationCache[group[0].groupNumber] = groupOBB.invMatrix;

        // Apply the representative OBB to each node in the group
        for (auto& node : group)
        {
            glm::vec3 nodeCenter = glm::vec3(node.obb.mid.x, node.obb.mid.y, node.obb.mid.z);
            glm::mat4 nodeTranslationMatrix = glm::translate(glm::mat4(1.0f), nodeCenter);

            // Compute the final transformation matrix for the node
            glm::mat4 finalNodeMatrix = nodeTranslationMatrix * rotationMatrix * scaleMatrix;

            Node tempNode = node;
            tempNode.obb = groupOBB;
            tempNode.obb.mid = node.obb.mid;
            tempNode.obb.invMatrix = glm::inverse(finalNodeMatrix);
            m_nodes[tempNode.index] = tempNode;

            m_clusterOBBs.push_back(tempNode.obb);
        }

        // For OBB visualization (uncomment if needed)
//            m_clusterOBBs[group[0].groupNumber] = groupOBB;
    }
    std::cout << "Cached transformations" << std::endl;

}


int core::obb::ObbTree::calculateMaxLeafDepth(const core::obb::Node *node, int depth) const
{
    if (node->isLeaf())
    {
        return depth;
    }

    int leftDepth = calculateMaxLeafDepth(&m_nodes[node->leftFrom], depth + 1);
    int rightDepth = calculateMaxLeafDepth(&m_nodes[node->leftFrom + 1], depth + 1);

    return std::max(leftDepth, rightDepth);
}

int core::obb::ObbTree::calculateMinLeafDepth(const core::obb::Node *node, int depth) const
{
    if (node->isLeaf())
    {
        return depth;
    }

    int leftDepth = calculateMinLeafDepth(&m_nodes[node->leftFrom], depth + 1);
    int rightDepth = calculateMinLeafDepth(&m_nodes[node->leftFrom + 1], depth + 1);

    return std::min(leftDepth, rightDepth);
}

void core::obb::ObbTree::collectLeafDepths(const core::obb::Node *node, int currentDepth)
{
    if (node->isLeaf())
    {
        m_leafDepths.push_back(currentDepth);
        return;
    }

    collectLeafDepths(&m_nodes[node->leftFrom], currentDepth + 1);
    collectLeafDepths(&m_nodes[node->leftFrom + 1], currentDepth + 1);
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
    glm::mat4 scaleMatrix = glm::scale(glm::mat4(1.0f),
                                       2.0f * (glm::vec3(node->obb.ext.x, node->obb.ext.y, node->obb.ext.z)));

    // Rotation matrix
    glm::mat4 rotationMatrix = glm::mat4(
            glm::vec4(glm::vec3(node->obb.v0.x, node->obb.v0.y, node->obb.v0.z), 0.0f),
            glm::vec4(glm::vec3(node->obb.v1.x, node->obb.v1.y, node->obb.v1.z), 0.0f),
            glm::vec4(glm::vec3(node->obb.v2.x, node->obb.v2.y, node->obb.v2.z), 0.0f),
            glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)
    );

    // Translation matrix
    glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f),
                                                 glm::vec3(node->obb.mid.x, node->obb.mid.y, node->obb.mid.z));

    // Calculate the inverse of the transformation matrix
    node->obb.invMatrix = glm::inverse(translationMatrix * rotationMatrix * scaleMatrix);
}

void core::obb::ObbTree::triangleIntersection(const core::obb::Node *const node, core::Ray &ray)
{
    for (int j = node->leftFrom; j < (node->leftFrom + node->count); j++)
    {
        core::intersect(m_triangles[j], ray);
    }
}

void printMatricesSideBySide(const glm::mat4x4 &matrix1, const std::string &name1,
                             const glm::mat4x4 &matrix2, const std::string &name2)
{
    std::cout << std::setw(50) << name1 << std::setw(50) << name2 << std::endl;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            std::cout << std::setw(10) << std::fixed << std::setprecision(4) << matrix1[i][j] << " ";
        }
        std::cout << "    ";
        for (int j = 0; j < 4; ++j)
        {
            std::cout << std::setw(10) << std::fixed << std::setprecision(4) << matrix2[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

void core::obb::ObbTree::intersectInternalNodes(const Node *node, core::Ray &ray, float &outT)
{
    Ray tempRay = ray;
    glm::vec4 rayOriginalLocal = (node->obb.invMatrix) * glm::vec4(tempRay.o, 1.0f);

    if (node->isLeaf() && m_useClustering)
    {
        if (m_cachedClusterRaydirs[node->groupNumber] == glm::vec3(0.0f))
        {
            m_cachedClusterRaydirs[node->groupNumber] = (node->obb.invMatrix) * glm::vec4(tempRay.d, 0);
        }
        glm::vec3 rayDirectionLocal = m_cachedClusterRaydirs[node->groupNumber];
        tempRay.o = glm::vec3(rayOriginalLocal.x, rayOriginalLocal.y, rayOriginalLocal.z);
        tempRay.rd = glm::vec3(1.0f / rayDirectionLocal.x, 1.0f / rayDirectionLocal.y, 1.0f / rayDirectionLocal.z);
    } else
    {
        glm::vec3 rayDirectionLocal = (node->obb.invMatrix) * glm::vec4(tempRay.d, 0.0f);
        tempRay.o = glm::vec3(rayOriginalLocal.x, rayOriginalLocal.y, rayOriginalLocal.z);
        tempRay.rd = glm::vec3(1.0f / rayDirectionLocal.x, 1.0f / rayDirectionLocal.y, 1.0f / rayDirectionLocal.z);
    }

    outT = core::intersectAABB(m_unitAABB, tempRay);
}

float core::obb::ObbTree::intersectInternalNodes4x4(const Node *node, core::Ray4x4 &rays,
                               const std::vector<glm::vec3> &cachedClusterRaydirs) const
{
    core::Ray4x4 tempRay = rays;

    // load rays origin into vectors
    //    Ray tempRay = ray;
    //    glm::vec4 rayOriginalLocal = (node->obb.invMatrix) * glm::vec4(tempRay.o, 1.0f);
    
    // Column 0
    __m256 m00 = _mm256_set1_ps(node->obb.invMatrix[0][0]);
    __m256 m10 = _mm256_set1_ps(node->obb.invMatrix[1][0]);
    __m256 m20 = _mm256_set1_ps(node->obb.invMatrix[2][0]);
    __m256 m30 = _mm256_set1_ps(node->obb.invMatrix[3][0]);

    // Column 1
    __m256 m01 = _mm256_set1_ps(node->obb.invMatrix[0][1]);
    __m256 m11 = _mm256_set1_ps(node->obb.invMatrix[1][1]);
    __m256 m21 = _mm256_set1_ps(node->obb.invMatrix[2][1]);
    __m256 m31 = _mm256_set1_ps(node->obb.invMatrix[3][1]);

    // Column 2
    __m256 m02 = _mm256_set1_ps(node->obb.invMatrix[0][2]);
    __m256 m12 = _mm256_set1_ps(node->obb.invMatrix[1][2]);
    __m256 m22 = _mm256_set1_ps(node->obb.invMatrix[2][2]);
    __m256 m32 = _mm256_set1_ps(node->obb.invMatrix[3][2]);

    // Column 3
    __m256 m03 = _mm256_set1_ps(node->obb.invMatrix[0][3]);
    __m256 m13 = _mm256_set1_ps(node->obb.invMatrix[1][3]);
    __m256 m23 = _mm256_set1_ps(node->obb.invMatrix[2][3]);
    __m256 m33 = _mm256_set1_ps(node->obb.invMatrix[3][3]);

    // Process both halves (0-7 and 8-15)
    for (int i = 0; i < 2; i++)
    {
        // Load ray origins
        __m256 ox_x8 = _mm256_load_ps(tempRay.ox_x8.data() + i * 8);
        __m256 oy_x8 = _mm256_load_ps(tempRay.oy_x8.data() + i * 8);
        __m256 oz_x8 = _mm256_load_ps(tempRay.oz_x8.data() + i * 8);
        __m256 ow_x8 = _mm256_set1_ps(1.0f);

        // Transform the ray origins
        __m256 result_x = _mm256_add_ps(
                _mm256_add_ps(_mm256_mul_ps(m00, ox_x8), _mm256_mul_ps(m10, oy_x8)),
                _mm256_add_ps(_mm256_mul_ps(m20, oz_x8), _mm256_mul_ps(m30, ow_x8)));

        __m256 result_y = _mm256_add_ps(
                _mm256_add_ps(_mm256_mul_ps(m01, ox_x8), _mm256_mul_ps(m11, oy_x8)),
                _mm256_add_ps(_mm256_mul_ps(m21, oz_x8), _mm256_mul_ps(m31, ow_x8)));

        __m256 result_z = _mm256_add_ps(
                _mm256_add_ps(_mm256_mul_ps(m02, ox_x8), _mm256_mul_ps(m12, oy_x8)),
                _mm256_add_ps(_mm256_mul_ps(m22, oz_x8), _mm256_mul_ps(m32, ow_x8)));

        // Store the transformed origins back into tempRay
        _mm256_store_ps(tempRay.ox_x8.data() + i * 8, result_x);
        _mm256_store_ps(tempRay.oy_x8.data() + i * 8, result_y);
        _mm256_store_ps(tempRay.oz_x8.data() + i * 8, result_z);
    }

    if (node->isLeaf() && m_useClustering)
    {
        glm::vec3 rayDirectionLocal = cachedClusterRaydirs[node->groupNumber];
        tempRay.rd = glm::vec3(1.0f / rayDirectionLocal.x, 1.0f / rayDirectionLocal.y, 1.0f / rayDirectionLocal.z);
    } else
    {
        glm::vec3 rayDirectionLocal = (node->obb.invMatrix) * glm::vec4(tempRay.d, 0.0f);
        tempRay.rd = glm::vec3(1.0f / rayDirectionLocal.x, 1.0f / rayDirectionLocal.y, 1.0f / rayDirectionLocal.z);
    }

    return core::intersect4x4(m_unitAABB, tempRay);
}

