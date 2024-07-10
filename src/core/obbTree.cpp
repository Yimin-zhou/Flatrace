#include <numeric>
#include <iostream>
#include <chrono>
#include <random>
#include <algorithm>
#include <mlpack/methods/kmeans/kmeans.hpp>
#include <mlpack/core.hpp>
#include <dlib/clustering.h>
#include <dlib/matrix.h>

#include "obbTree.h"
#include "src/utils/globalState.h"
#include "Tracy.hpp"

core::obb::ObbTree::ObbTree(const std::vector<Triangle> &triangles, bool useClustering, int num_clusters)
        :
        _failed(false),
        _triangles(triangles),
        _triangleIds(triangles.size()),
        _triangleCentroids(triangles.size()),
        _unitAABB(glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(0.5f, 0.5f, 0.5f)),
        m_nGroup(num_clusters),
        m_clusterOBBs(m_nGroup),
        m_useClustering(useClustering)
{
    std::iota(_triangleIds.begin(), _triangleIds.end(), 0);

    std::transform(triangles.begin(), triangles.end(), _triangleCentroids.begin(), [](const Triangle &t)
    {
        return (t.vertices[0] + t.vertices[1] + t.vertices[2]) / 3.0f;
    });

    _nodes.reserve(triangles.size() * 2 - 1);
    _root = &_nodes.emplace_back(0, triangles.size(), 0);

    splitNode(_root);

    // BVH stats
    _maxDepth = calculateMaxLeafDepth(_root);
    int minDepth = calculateMinLeafDepth(_root);
    collectLeafDepths(_root);

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

bool core::obb::ObbTree::traversal(core::Ray &ray, const int maxIntersections, const std::vector<glm::vec3>& cachedClusterRaydirs, bool useRaycaching)
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
                intersectInternalNodes(left, ray, t_left, cachedClusterRaydirs, useRaycaching);
                intersectInternalNodes(right, ray, t_right, cachedClusterRaydirs, useRaycaching);

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

//    for (int i = node->leftFrom; i < (node->leftFrom + node->count); i++)
//    {
//        for (const glm::vec3 &v: getTriangle(i).vertices)
//        {
//            node->bbox.min = glm::min(node->bbox.min, v);
//            node->bbox.max = glm::max(node->bbox.max, v);
//        }
//    }

    if (node->count > LEAF_SIZE)
    {
//        auto split_plane = splitPlaneMid(node, 32);
        auto split_plane = splitPlaneSAH(node, node->leftFrom, node->count, 32);

        if (split_plane)
        {
            auto split_index = partition(node->leftFrom, node->count, *split_plane);

            if (split_index)
            {
                int left_index = _nodes.size();
                _nodes.emplace_back(node->leftFrom, *split_index - node->leftFrom, left_index);
                _nodes.emplace_back(*split_index, node->leftFrom + node->count - *split_index, left_index + 1);

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
        break;
    }

    return best_plane;
}

std::optional<core::Plane>
core::obb::ObbTree::splitPlaneSAH(const Node* const node, const int from, const int count, int maxSplitsPerDimension) const
{
    glm::vec3 mid = glm::vec3(node->obb.mid.x, node->obb.mid.y, node->obb.mid.z);
    glm::vec3 axis0 = glm::vec3(node->obb.v0.x, node->obb.v0.y, node->obb.v0.z);
    glm::vec3 axis1 = glm::vec3(node->obb.v1.x, node->obb.v1.y, node->obb.v1.z);
    glm::vec3 axis2 = glm::vec3(node->obb.v2.x, node->obb.v2.y, node->obb.v2.z);

    std::array<SplitDim, 3> split_dims =
            {
                    SplitDim{axis0, mid - axis0 * node->obb.ext.x, mid + axis0 * node->obb.ext.x},
                    SplitDim{axis1, mid - axis1 * node->obb.ext.y, mid + axis1 * node->obb.ext.y},
                    SplitDim{axis2, mid - axis2 * node->obb.ext.z, mid + axis2 * node->obb.ext.z}
            };

    const int splitsPerDimension = std::min(count, maxSplitsPerDimension);

    float best_cost = std::numeric_limits<float>::infinity();
    std::optional<Plane> best_plane;

    for (const SplitDim& split_dim : split_dims)
    {
        std::vector<SplitBin> bins(splitsPerDimension + 1);

        const float dim_width = glm::distance(split_dim.minPoint, split_dim.maxPoint);
        const float bin_width = dim_width / bins.size();

        if (bin_width <= 1e-6)
        {
            continue;
        }

        // Debug: Log split dimension information
        std::cout << "Dimension width: " << dim_width << ", Bin width: " << bin_width << std::endl;

        for (int triangle_index = from; triangle_index < (from + count); triangle_index++)
        {
            const glm::vec3& triangle_centroid = getCentroid(triangle_index);
            const float triangle_bin_offset = glm::dot(triangle_centroid - mid, split_dim.axis)  - glm::dot(split_dim.minPoint - mid, split_dim.axis);
            const int bin_index = std::min<int>(triangle_bin_offset / bin_width, bins.size() - 1);

            core::Triangle tempTri = getTriangle(triangle_index);
            DiTO::Vector<float> vertex_0 = DiTO::Vector<float>(tempTri.vertices[0].x, tempTri.vertices[0].y, tempTri.vertices[0].z);
            DiTO::Vector<float> vertex_1 = DiTO::Vector<float>(tempTri.vertices[1].x, tempTri.vertices[1].y, tempTri.vertices[1].z);
            DiTO::Vector<float> vertex_2 = DiTO::Vector<float>(tempTri.vertices[2].x, tempTri.vertices[2].y, tempTri.vertices[2].z);

            std::vector<DiTO::Vector<float>> vertices = {vertex_0, vertex_1, vertex_2};

            // Compute the OBB using the DiTO algorithm, if there are vertices present
            if (!vertices.empty())
            {
                DiTO::DiTO_14(vertices.data(), vertices.size(), bins[bin_index].obb);
            }

            bins[bin_index].trianglesIn++;

            // Debug: Log triangle assignment to bin
            std::cout << "Triangle " << triangle_index << " -> bin " << bin_index << std::endl;
        }

        float left_sum = 0.0f;
        float right_sum = 0.0f;

        SplitBin& bin_left_temp = bins[0];
        SplitBin& bin_right_temp = bins[bins.size() - 1];

        DiTO::OBB<float> left_obb = bin_left_temp.obb;
        DiTO::OBB<float> right_obb = bin_right_temp.obb;

        for (int i = 0; i < splitsPerDimension; i++)
        {
            const int bin_index_left = i;
            const int bin_index_right = bins.size() - i - 1;

            SplitBin& bin_left = bins[bin_index_left];
            SplitBin& bin_right = bins[bin_index_right];
            left_sum += bin_left.trianglesIn;
            left_obb = left_obb.extended(bin_left.obb);

            bin_left.trianglesLeft = left_sum;
            bin_left.areaLeft = left_obb.area();

            right_sum += bin_right.trianglesIn;
            right_obb = right_obb.extended(bin_right.obb);

            bins[bin_index_right - 1].trianglesRight = right_sum;
            bins[bin_index_right - 1].areaRight = right_obb.area();

            // Debug: Log left and right bin accumulations
            std::cout << "Bin " << bin_index_left << ": left_sum = " << left_sum << ", left_area = " << bin_left.areaLeft << std::endl;
            std::cout << "Bin " << bin_index_right << ": right_sum = " << right_sum << ", right_area = " << bin_right.areaRight << std::endl;
        }

        for (int plane_index = 1; plane_index < splitsPerDimension; plane_index++)
        {
            const float d = glm::dot(split_dim.minPoint - mid, split_dim.axis) + plane_index * bin_width;

            const int n_left = bins[plane_index - 1].trianglesLeft;
            const int n_right = bins[plane_index].trianglesRight + bins[plane_index].trianglesIn;

            const float cost_left = (n_left != 0 ? n_left * bins[plane_index - 1].areaLeft : 0.0f);
            const float cost_right = (n_right != 0 ? n_right * bins[plane_index].areaRight : 0.0f);

            const float cost = cost_left + cost_right;

            // Debug: Log SAH cost calculations
            std::cout << "Plane " << plane_index << ": d = " << d << ", n_left = " << n_left << ", cost_left = " << cost_left << ", n_right = " << n_right << ", cost_right = " << cost_right << ", total_cost = " << cost << std::endl;

            if (cost < best_cost)
            {
                best_plane = Plane(mid + split_dim.axis * d, split_dim.axis);
                best_cost = cost;

                // Debug: Log best plane update
                std::cout << "Best plane updated: d = " << d << ", best_cost = " << best_cost << std::endl;
            }

            if (plane_index == 18) break;
        }
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

std::vector<std::vector<core::obb::Node>> core::obb::ObbTree::clusterOBBsKmeans(int num_clusters)
{
    std::vector<std::vector<core::obb::Node>> finalGroups(num_clusters);
    std::vector<core::obb::Node> leafNodes;
    std::vector<size_t> leafNodeIndices;

    // Extract leaf nodes and their indices
    for (size_t i = 0; i < _nodes.size(); ++i)
    {
        if (_nodes[i].isLeaf())
        {
            leafNodes.push_back(_nodes[i]);
            leafNodeIndices.push_back(i);
        }
    }

    std::vector<Eigen::Matrix<float, Eigen::Dynamic, 1>> data;
    for (const auto& node : leafNodes)
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
        _nodes[leafNodeIndices[i]].groupNumber = assignments[i];
        clusters[assignments[i]].push_back(leafNodes[i]);
    }

    std::cerr << "Clustered " << leafNodes.size() << " OBBs into " << num_clusters << " clusters." << std::endl;

    return clusters;
}

std::vector<std::vector<core::obb::Node>> core::obb::ObbTree::clusterOBBsMeanshift()
{
//    std::vector<std::vector<core::obb::Node>> clusters;
//    std::vector<core::obb::Node> leafNodes;
//    std::vector<size_t> leafNodeIndices;
//
//    // Extract leaf nodes and their indices
//    for (size_t i = 0; i < _nodes.size(); ++i)
//    {
//        if (_nodes[i].isLeaf())
//        {
//            leafNodes.push_back(_nodes[i]);
//            leafNodeIndices.push_back(i);
//        }
//    }
//
//    std::vector<dlib::matrix<double, 0, 1>> samples;
//    for (const auto& node : leafNodes)
//    {
//        Eigen::Matrix<float, Eigen::Dynamic, 1> flattened = DiTO::flatten(node.obb);
//        dlib::matrix<double, 0, 1> sample(flattened.size());
//        for (size_t i = 0; i < flattened.size(); ++i)
//        {
//            sample(i) = flattened(i);
//        }
//        samples.push_back(sample);
//    }
//
//    // Perform Mean Shift clustering
//    std::vector<dlib::matrix<double, 0, 1>> initial_centers;
//    double bandwidth = 1.0; // Define a suitable bandwidth parameter for Mean Shift
//    dlib::mean_shift(samples, initial_centers, bandwidth);
//
//    // Assign each sample to the nearest center
//    std::vector<size_t> assignments(samples.size());
//    for (size_t i = 0; i < samples.size(); ++i)
//    {
//        double min_distance = std::numeric_limits<double>::max();
//        size_t closest_cluster = 0;
//
//        for (size_t j = 0; j < initial_centers.size(); ++j)
//        {
//            double distance = dlib::length(samples[i] - initial_centers[j]);
//            if (distance < min_distance)
//            {
//                min_distance = distance;
//                closest_cluster = j;
//            }
//        }
//        assignments[i] = closest_cluster;
//    }
//
//    // Create clusters and update the original nodes with new group numbers
//    size_t num_clusters = initial_centers.size();
//    clusters.resize(num_clusters);
//    for (size_t i = 0; i < assignments.size(); ++i)
//    {
//        leafNodes[i].groupNumber = assignments[i];
//        _nodes[leafNodeIndices[i]].groupNumber = assignments[i];
//        clusters[assignments[i]].push_back(leafNodes[i]);
//    }
//
//    std::cerr << "Clustered " << leafNodes.size() << " OBBs into " << num_clusters << " clusters using Mean Shift." << std::endl;
//
//    return clusters;
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
        DiTO::OBB<float> tempOBB;
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
        DiTO::DiTO_14(tempVertices.data(), tempVertices.size(), tempOBB);
        glm::vec3 groupCenter = glm::vec3(tempOBB.mid.x, tempOBB.mid.y, tempOBB.mid.z);

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
        for (const auto& node : group)
        {
            glm::vec3 nodeCenter = glm::vec3(node.obb.mid.x, node.obb.mid.y, node.obb.mid.z);
            glm::mat4 nodeTranslationMatrix = glm::translate(glm::mat4(1.0f), nodeCenter);

            // Compute the final transformation matrix for the node
            glm::mat4 finalNodeMatrix = nodeTranslationMatrix * rotationMatrix * scaleMatrix;

            Node tempNode = node;
            tempNode.obb = groupOBB;
            tempNode.obb.mid = node.obb.mid;
            tempNode.obb.invMatrix = glm::inverse(finalNodeMatrix);
            _nodes[tempNode.index] = tempNode;
        }

        // For OBB visualization (uncomment if needed)
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

void printMatricesSideBySide(const glm::mat4x4& matrix1, const std::string& name1,
                             const glm::mat4x4& matrix2, const std::string& name2)
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

void core::obb::ObbTree::intersectInternalNodes(const Node *node, core::Ray &ray, float& outT,
                                                const std::vector<glm::vec3>& cachedClusterRaydirs, bool useRaycaching)
{
    Ray tempRay = ray;

    // Use grouped transformations for leaf nodes
    if (node->isLeaf() && m_useClustering && useRaycaching)
    {
        glm::vec4 rayOriginalLocal = (node->obb.invMatrix) * glm::vec4(tempRay.o, 1.0f);
        glm::vec3 rayDirectionLocal = cachedClusterRaydirs[node->groupNumber];
        tempRay.o = glm::vec3(rayOriginalLocal.x, rayOriginalLocal.y, rayOriginalLocal.z);
        tempRay.rd = glm::vec3(1.0f / rayDirectionLocal.x, 1.0f / rayDirectionLocal.y, 1.0f / rayDirectionLocal.z);
    }
    else if (useRaycaching && !m_useClustering)
    {
        glm::vec4 rayOriginalLocal = (node->obb.invMatrix) * glm::vec4(tempRay.o, 1.0f);
        tempRay.o = glm::vec3(rayOriginalLocal.x, rayOriginalLocal.y, rayOriginalLocal.z);
        tempRay.rd = glm::vec3(1.0f / node->cachedRayDir.x, 1.0f / node->cachedRayDir.y, 1.0f / node->cachedRayDir.z);
    }
    else
    {   
        glm::vec4 rayOriginalLocal = (node->obb.invMatrix) * glm::vec4(tempRay.o, 1.0f);
        glm::vec3 rayDirectionLocal = (node->obb.invMatrix) * glm::vec4(tempRay.d, 0.0f);
        tempRay.o = glm::vec3(rayOriginalLocal.x, rayOriginalLocal.y, rayOriginalLocal.z);
        tempRay.rd = glm::vec3(1.0f / rayDirectionLocal.x, 1.0f / rayDirectionLocal.y, 1.0f / rayDirectionLocal.z);
    }

    outT = core::intersectAABB(_unitAABB, tempRay);
}


