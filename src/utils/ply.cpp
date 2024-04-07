#include <fstream>
#include "ply.h"

namespace utils::ply
{

    std::vector<core::Triangle> PlyInterpreter::read(const std::string &filepath)
    {
        loadFile(filepath);
        scaleModel(_triangles, glm::vec3(0.0005f));
        return _triangles;
    }

    void PlyInterpreter::loadFile(const std::string &filepath)
    {
        std::ifstream ss(filepath, std::ios::binary);
        if (ss.fail()) throw std::runtime_error("Failed to open " + filepath);

        tinyply::PlyFile file;
        file.parse_header(ss);

        auto vertices = file.request_properties_from_element("vertex", {"x", "y", "z"});
        auto faces = file.request_properties_from_element("face", {"vertex_indices"}, 3);
        file.read(ss);

        if (vertices)
        {
            const size_t numVertices = vertices->count;
            std::vector<float> verts(numVertices * 3);
            std::memcpy(verts.data(), vertices->buffer.get(), verts.size() * sizeof(float));

            for (size_t i = 0; i < numVertices; ++i)
            {
                _vertices.push_back({verts[i * 3], verts[i * 3 + 1], verts[i * 3 + 2]});
            }
        }

        if (faces)
        {
            const size_t numFaces = faces->count;
            auto faceBuffer = reinterpret_cast<const int32_t *>(faces->buffer.get());
            for (size_t i = 0; i < numFaces; ++i)
            {
                size_t idx = i * (faces->count + 1); // +1 for the vertex count at the beginning
                int numVertices = faceBuffer[idx];
                Face faceVertices;
                for (int j = 0; j < numVertices; ++j)
                {
                    faceVertices.push_back(faceBuffer[idx + 1 + j]); // +1 to skip vertex count
                }
                _faces.push_back(faceVertices);
            }
        }

        generateTriangles();
    }

    void PlyInterpreter::generateTriangles()
    {
        int triangleId = 0;
        for (const auto &face: _faces)
        {
            for (size_t i = 1; i < face.size() - 1; ++i)
            {
                const auto &v0 = _vertices[face[0]];
                const auto &v1 = _vertices[face[i]];
                const auto &v2 = _vertices[face[i + 1]];
                _triangles.emplace_back(triangleId++, glm::vec3(v0.x, v0.y, v0.z), glm::vec3(v1.x, v1.y, v1.z),
                                        glm::vec3(v2.x, v2.y, v2.z), 0);
            }
        }
    }

    void PlyInterpreter::scaleModel(std::vector<core::Triangle> &triangles, const glm::vec3 &scaleFactor)
    {
        for (auto &triangle: triangles)
        {
            for (auto &vertex: triangle.vertices)
            {
                vertex *= scaleFactor;
            }
            triangle.edges[0] = triangle.vertices[1] - triangle.vertices[0];
            triangle.edges[1] = triangle.vertices[2] - triangle.vertices[0];
            triangle.normal = glm::normalize(glm::cross(triangle.edges[0], triangle.edges[1]));
        }
    }

}