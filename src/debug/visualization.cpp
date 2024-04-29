#include "visualization.h"
#include "src/utils/globalState.h"

debug::Visualization::Visualization(const core::BVH &bvh) : m_inputBVH(bvh)
{
    int triangleId = 0;
    traversalNodes(m_inputBVH.getRoot(), m_triangles, triangleId);
}

debug::Visualization::Visualization(const core::obb::ObbTree &obbTree) : m_inputObbTree(obbTree)
{
    int triangleId = 0;
    traversalNodes(m_inputObbTree.getRoot(), m_triangles, triangleId);
}

void debug::Visualization::traversalNodes(const core::Node* node,
                                          std::vector<core::Triangle> &triangles, int &triangleId)
{
    if (node == nullptr)
    {
        return;
    }
    //     Only visualize the bounding box if it's a leaf node
    if (node->isLeaf())
    {
        // Visualize AABB
        glm::vec3 center = (node->bbox.min + node->bbox.max) * 0.5f;
        glm::vec3 dimensions = node->bbox.max - node->bbox.min;
        std::vector<core::Triangle> nodeTriangles = visualizeAABB(center, dimensions, triangleId);
        triangles.insert(triangles.end(), nodeTriangles.begin(), nodeTriangles.end());
    }
    else
    {
        // Recurse for children
        traversalNodes(&m_inputBVH.getNodes()[node->leftFrom], triangles, triangleId);
        traversalNodes(&m_inputBVH.getNodes()[node->leftFrom + 1], triangles, triangleId);
    }
}

void debug::Visualization::traversalNodes(const core::obb::Node *node, std::vector<core::Triangle> &triangles,
                                          int &triangleId)
{
    if (node == nullptr)
    {
        return;
    }
    //     Only visualize the bounding box if it's a leaf node
    if (node->isLeaf())
    {
        // Visualize OBB
        std::vector<core::Triangle> nodeTriangles = visualizeOBB(node->obb, triangleId);
        triangles.insert(triangles.end(), nodeTriangles.begin(), nodeTriangles.end());
    }
    else
    {
        // Recurse for children
        traversalNodes(&m_inputObbTree.getNodes()[node->leftFrom], triangles, triangleId);
        traversalNodes(&m_inputObbTree.getNodes()[node->leftFrom + 1], triangles, triangleId);
    }
}

std::vector<core::Triangle>
debug::Visualization::visualizeAABB(const glm::vec3 &center, const glm::vec3 &dimensions, int &triangleId) const
{
    std::vector<core::Triangle> triangles;
    glm::vec3 halfDimensions = dimensions * 0.5f;

    // Calculate vertices based on center and dimensions
    std::array<glm::vec3, 8> vertices ={
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

    for (const auto &face: faces)
    {
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

template<typename T>
std::vector<core::Triangle> debug::Visualization::visualizeOBB(const DiTO::OBB<T> &obb, int &triangleId) const
{
    std::vector<core::Triangle> triangles;

    // Convert OBB vectors to glm::vec3
    glm::vec3 mid = glm::vec3(obb.mid.x, obb.mid.y, obb.mid.z);
    glm::vec3 v0 = glm::vec3(obb.v0.x, obb.v0.y, obb.v0.z);
    glm::vec3 v1 = glm::vec3(obb.v1.x, obb.v1.y, obb.v1.z);
    glm::vec3 v2 = glm::vec3(obb.v2.x, obb.v2.y, obb.v2.z);
    glm::vec3 ext = glm::vec3(obb.ext.x, obb.ext.y, obb.ext.z);

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
    for (const auto &face: faces)
    {
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



