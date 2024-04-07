#pragma once

#include "src/core/types.h"
#include "third_party/tiny_ply/tinyply.h"

#include <vector>
#include <string>
#include <fstream>

namespace utils::ply
{
    using Vertex = glm::vec3;
    using Face = std::vector<int>;

    class PlyInterpreter
    {
    public:
        std::vector<core::Triangle> read(const std::string &filepath);

    private:
        std::vector<Vertex> _vertices;
        std::vector<Face> _faces;
        std::vector<core::Triangle> _triangles;

        void loadFile(const std::string &filepath);

        void generateTriangles();

        void scaleModel(std::vector<core::Triangle> &triangles, const glm::vec3 &scaleFactor);
    };
}
