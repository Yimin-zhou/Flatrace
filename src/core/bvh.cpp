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
#include <algorithm>
#include <mlpack/methods/kmeans/kmeans.hpp>

#ifdef IS_X86

#include <immintrin.h>

#else
#include <simde/x86/avx2.h>
#endif

namespace core
{

    BVH::BVH(const std::vector<Triangle> &triangles, bool useOBB, bool isHybrid, int binSize, bool useClustering, int nGroup)
            :
            m_failed(false),
            m_triangles(triangles),
            m_triangleIds(triangles.size()),
            m_triangleCentroids(triangles.size()),
            m_unitAABB(glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(0.5f, 0.5f, 0.5f)),
            m_useOBB(useOBB),
            m_binSize(binSize),
            m_useClustering(useClustering),
            m_nGroup(nGroup),
            m_isHybrid(isHybrid),
            m_cachedClusterRaydirs(nGroup)
    {
        std::iota(m_triangleIds.begin(), m_triangleIds.end(), 0);

        std::transform(triangles.begin(), triangles.end(), m_triangleCentroids.begin(), [](const Triangle &t)
        {
            return (t.vertices[0] + t.vertices[1] + t.vertices[2]) / 3.0f;
        });

        m_nodes.reserve(triangles.size() * 2 - 1);
        m_root = &m_nodes.emplace_back(0, triangles.size(), 0);

        splitNode(m_root);

        m_maxDepth = calculateMaxLeafDepth(m_root);
        int minDepth = calculateMinLeafDepth(m_root);
        collectLeafDepths(m_root);

//        std::cerr << "NODE STRUCT SIZE: " << sizeof(Node) << std::endl;
//        std::cerr << "BVH SIZE: " << _nodes.size() << std::endl;
//        std::cerr << "BVH MAX DEPTH: " << _maxDepth << std::endl;
//        std::cerr << "BVH MIN DEPTH: " << minDepth << std::endl;

        // Re-order triangles such that triangles for each node are adjacent in memory again. This should improve
        // data locality and avoids having to use indirection when iterating triangles for intersection
        linearize();

        if (m_useClustering && m_useOBB)
        {
            // Kmeans
            m_clusteredNodes = clusterOBBsKmeans(m_nGroup);
            cacheTransformations();
        }
    }

// Linearize triangle indices
    void BVH::linearize()
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


    int BVH::calculateMaxLeafDepth(const Node *node, int depth) const
    {
        if (node->isLeaf())
        {
            return depth;
        }

        int leftDepth = calculateMaxLeafDepth(&m_nodes[node->leftFrom], depth + 1);
        int rightDepth = calculateMaxLeafDepth(&m_nodes[node->leftFrom + 1], depth + 1);

        return std::max(leftDepth, rightDepth);
    }

    int BVH::calculateMinLeafDepth(const Node *node, int depth) const
    {
        if (node->isLeaf())
        {
            return depth;
        }

        int leftDepth = calculateMinLeafDepth(&m_nodes[node->leftFrom], depth + 1);
        int rightDepth = calculateMinLeafDepth(&m_nodes[node->leftFrom + 1], depth + 1);

        return std::min(leftDepth, rightDepth);
    }

    void BVH::collectLeafDepths(const Node *node, int currentDepth)
    {
        if (node->isLeaf())
        {
            m_leafDepths.push_back(currentDepth);
            return;
        }

        collectLeafDepths(&m_nodes[node->leftFrom], currentDepth + 1);
        collectLeafDepths(&m_nodes[node->leftFrom + 1], currentDepth + 1);
    }


    bool BVH::traversal(Ray &ray, const int maxIntersections)
    {
        ZoneScopedN("AABB BVH Traversal");

        const Node *node_stack[m_nodes.size()];

        if (core::intersectAABB(m_root->bbox, ray) == INF)
        {
            return false;
        }

        for (int i = 0; i < maxIntersections; i++)
        {
            int stack_pointer = 0;

            node_stack[stack_pointer++] = m_root;

            while (stack_pointer != 0)
            {
                const Node *const node = node_stack[--stack_pointer];

                ray.bvh_nodes_visited++;
                if (node->isLeaf())
                {
                    triangleIntersection(node, ray);
                } else
                {
                    const Node *left = &m_nodes[node->leftFrom];
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
                        core::intersect4x4(m_triangles[i], rays);
                    }
                } else
                {
                    const Node *child_0 = &m_nodes[node->leftFrom];
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

    bool BVH::traversalOBB(Ray &ray, const int maxIntersections)
    {
        ZoneScopedN("OBB in AABB BVH Traversal");

        const Node *node_stack[m_nodes.size()];

        if (core::intersectAABB(m_root->bbox, ray) == INF)
        {
            return false;
        }

        for (int i = 0; i < maxIntersections; i++)
        {
            int stack_pointer = 0;

            node_stack[stack_pointer++] = m_root;

            while (stack_pointer != 0)
            {
                const Node *const node = node_stack[--stack_pointer];

                ray.bvh_nodes_visited++;
                if (node->isLeaf())
                {

                    for (int j = node->leftFrom; j < (node->leftFrom + node->count); j++)
                    {
                        core::intersect(m_triangles[j], ray);
                    }
                } else
                {
                    ZoneScopedN("Internal OBB Intersect");

                    const Node *left = &m_nodes[node->leftFrom];
                    const Node *right = left + 1;

                    // transform ray to obb space for both left and right node
                    float t_left;
                    float t_right;
                    intersectInternalNodesOBB(left, ray, t_left);
                    intersectInternalNodesOBB(right, ray, t_right);

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

    bool BVH::traversalHybrid(Ray &ray, const int maxIntersections)
    {
        ZoneScopedN("Hybrid BVH Traversal");

        const Node *node_stack[m_nodes.size()];

        if (core::intersectAABB(m_root->bbox, ray) == INF)
        {
            return false;
        }

        for (int i = 0; i < maxIntersections; i++)
        {
            int stack_pointer = 0;

            node_stack[stack_pointer++] = m_root;

            while (stack_pointer != 0)
            {
                const Node *const node = node_stack[--stack_pointer];

                ray.bvh_nodes_visited++;
                if (node->isLeaf())
                {
                    triangleIntersection(node, ray);
                } else
                {
                    const Node *left = &m_nodes[node->leftFrom];
                    const Node *right = left + 1;

                    float t_left = 0;
                    float t_right = 0;

                    if (left->obbFlag)
                    {
                        intersectInternalNodesOBB(left, ray, t_left);
                    } else
                    {
                        intersectInternalNodesAABB(left, ray, t_left);
                    }

                    if (right->obbFlag)
                    {
                        intersectInternalNodesOBB(right, ray, t_right);
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

    std::vector<std::vector<Node>> BVH::clusterOBBsKmeans(int num_clusters)
    {
        std::vector<std::vector<core::Node>> finalGroups(num_clusters);
        std::vector<core::Node> leafNodes;
        std::vector<size_t> leafNodeIndices;

        // Extract leaf nodes and their indices
        for (size_t i = 0; i < m_nodes.size(); ++i)
        {
            if (m_nodes[i].isLeaf() && m_nodes[i].obbFlag)
            {
                leafNodes.push_back(m_nodes[i]);
                leafNodeIndices.push_back(i);
            }
        }
        m_enabledOBBLeafSize = leafNodes.size();

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
        std::vector<std::vector<core::Node>> clusters(num_clusters);
        for (size_t i = 0; i < assignments.n_elem; ++i)
        {
            leafNodes[i].groupNumber = assignments[i];
            m_nodes[leafNodeIndices[i]].groupNumber = assignments[i];
            clusters[assignments[i]].push_back(leafNodes[i]);
        }

        std::cerr << "Clustered " << leafNodes.size() << " OBBs into " << num_clusters << " clusters." << std::endl;

        return clusters;
    }

    void BVH::cacheTransformations()
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


    Node *BVH::splitNode(Node *const node)
    {
        // generate obb
        if (m_useOBB)
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
        if (m_isHybrid)
        {
            if (node->bbox.area() > node->obb.area())
            {
                node->obbFlag = true;
            }
        } else if(m_useOBB)
        {
            node->obbFlag = true;
        }

        // Subdivide if this is not a leaf node (getTriangle count below cutoff)
        if (node->count > TracerState::LEAF_SIZE)
        {
            const std::optional<Plane> split_plane = splitPlaneSAH(node, node->leftFrom, node->count, m_binSize);

            if (split_plane)
            {
                const std::optional<int> split_index = partition(node->leftFrom, node->count, *split_plane);

                if (split_index)
                {
                    const int left_index = m_nodes.size();
                    const int right_index = left_index + 1;

                    const int left_count = *split_index - node->leftFrom;
                    const int right_count = node->count - left_count;

                    m_nodes.emplace_back(node->leftFrom, left_count, left_index);
                    m_nodes.emplace_back(*split_index, right_count, right_index);

                    splitNode(&m_nodes[left_index]);
                    splitNode(&m_nodes[right_index]);

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
                std::swap(m_triangleIds[left_to], m_triangleIds[--right_from]);
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

    void BVH::intersectInternalNodesOBB(const Node *node, Ray &ray, float &outT)
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
}