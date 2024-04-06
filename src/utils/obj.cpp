// Worlds' stupidest .OBJ parser

#include "obj.h"

#include <boost/spirit/include/qi.hpp>
#include <boost/fusion/include/adapt_struct.hpp>

#include <fstream>
#include <variant>
#include <array>
#include <map>
#include <filesystem>

#include "third_party/fmt/format.h"
#include "third_party/tiny_obj/tiny_obj_loader.h"

namespace qi = boost::spirit::qi;

namespace fs = std::filesystem;

namespace utils::Obj {

//
// Boost.Spirit parser for .OBJ files, only parses what we need/want, ignores everything else
//

struct Vertex
{
  float x;
  float y;
  float z;
};

    void scaleModel(std::vector<core::Triangle>& triangles, const glm::vec3& scaleFactor) {
        for (auto& triangle : triangles) {
            // Scale each vertex in the triangle
            for (auto& vertex : triangle.vertices) {
                vertex *= scaleFactor;
            }
            // If you are storing edges and they are used elsewhere, you should update them as well
            triangle.edges[0] = triangle.vertices[1] - triangle.vertices[0];
            triangle.edges[1] = triangle.vertices[2] - triangle.vertices[0];
            // Update the normal if it's used elsewhere, since scaling can change its magnitude (if non-uniform scaling is used)
            triangle.normal = glm::normalize(glm::cross(triangle.edges[0], triangle.edges[1]));
        }
    }

struct Material { std::string name; };

using Face = std::vector<int>;
using Statement = std::variant<std::string, Vertex, Face, Material>;

struct ObjLineParser : public qi::grammar<std::string::const_iterator, std::vector<Statement>(), qi::blank_type>
{
  template<typename T>
  using Rule = qi::rule<iterator_type, T(), qi::blank_type>;

  ObjLineParser() : ObjLineParser::base_type(obj)
  {
    label = *(qi::char_ - qi::eol);

    vertex = qi::lexeme['v'] >> (qi::double_ > qi::double_ > qi::double_ > qi::eol);
    vertexIndex = qi::int_ >> qi::omit[ *('/' >> -qi::int_) ];
    face = qi::lexeme['f'] >> (qi::repeat(3, 4)[vertexIndex] > qi::eol);

    material = qi::lexeme["usemtl"] >> (label > qi::eol);

    unknown = *(qi::char_ - qi::eol) >> qi::eol;

    line = (vertex | face | material | qi::omit[unknown]);

    obj = *line >> qi::eoi;
  }

  Rule<std::string> label;
  Rule<Vertex> vertex;
  Rule<int> vertexIndex;
  Rule<Face> face;
  Rule<Material> material;
  Rule<std::string> unknown;
  Rule<Statement> line;
  Rule<std::vector<Statement>> obj;
};

// Visitor class that interprets parsed OBJ statements, to turn them into getTriangle soup
class ObjInterpreter
{
  public:
    void operator()(std::string) {};

    void operator()(const Obj::Vertex &vertex)
    {
      _vertices.push_back({ vertex.x, vertex.y, vertex.z });
    }

    void operator()(const Obj::Face &face)
    {
      _triangles.emplace_back(static_cast<int>(_triangles.size()), _vertices[face[0] - 1], _vertices[face[1] - 1], _vertices[face[2] - 1], _currentMaterial);

      if (face.size() == 4)
      {
        _triangles.emplace_back(static_cast<int>(_triangles.size()), _vertices[face[0] - 1], _vertices[face[2] - 1], _vertices[face[3] - 1], _currentMaterial);
      }
    }

    void operator()(const Obj::Material &material)
    {
      const auto &[m, existed] = _materials.insert({ material.name, static_cast<int>(_materials.size()) });

      _currentMaterial = m->second;
    }

    const std::vector<core::Triangle> &getTriangles() const { return _triangles; }

  private:
    std::vector<glm::vec3> _vertices;
    std::vector<core::Triangle> _triangles;

    std::map<std::string, int> _materials;
    int _currentMaterial = 0;
};

// Read .OBJ file contents and return its contents as triangle soup, optionally normalizing vertex
// XYZ coordinates to [0..1] range
std::vector<core::Triangle> read(const std::string &filename, const bool normalize)
{
  std::ifstream in(filename, std::ios::in);

  if (!in.is_open())
  {
    throw std::runtime_error("Failed to open file");
  }

  const std::string obj_text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());

  // The Boost.Spirit grammar for the OBJ parser will simply go through the file line-by-line, and
  // for any recognized line create a 'Statement', which is a variant type that holds the parsed
  // data for that line. Lines we don't care about are simply ignored.
  std::vector<Statement> statements;

  try
  {
    std::string::const_iterator it = obj_text.begin();

    const bool parsed = qi::phrase_parse(it, obj_text.end(), ObjLineParser(), qi::blank, statements);

    if (parsed && (it == obj_text.end()))
    {
      // File parsed OK, now interpret the parsed statements
      ObjInterpreter obj_interpreter;

      std::for_each(statements.begin(), statements.end(), [&obj_interpreter](const Obj::Statement &s) { std::visit(obj_interpreter, s); });

      std::vector<core::Triangle> triangles = obj_interpreter.getTriangles();

      if (normalize)
      {
        glm::vec3 xyz_min = { core::INF,  core::INF, core::INF };
        glm::vec3 xyz_max = { -core::INF,  -core::INF, -core::INF };

        for (const core::Triangle &triangle : triangles)
        {
          for (const glm::vec3 &v : triangle.vertices)
          {
            xyz_min = glm::min(xyz_min, v);
            xyz_max = glm::max(xyz_max, v);
          }
        }

        const glm::vec3 center = (xyz_min + xyz_max) / 2.0f;
        const glm::vec3 range = (xyz_max - xyz_min);
        const float max_dim = std::max(range.x, std::max(range.y, range.z));

        std::transform(triangles.begin(), triangles.end(), triangles.begin(), [center, max_dim](core::Triangle &triangle)
        {
          const glm::vec3 v0 = (triangle.vertices[0] - center) / max_dim;
          const glm::vec3 v1 = (triangle.vertices[1] - center) / max_dim;
          const glm::vec3 v2 = (triangle.vertices[2] - center) / max_dim;

          return core::Triangle{ triangle.id, v0, v1, v2, triangle.material };
        });
      }

//      scaleModel(triangles, glm::vec3(0.00035f));
      return triangles;
    }
    else
    {
      throw std::runtime_error("OBJ parse error");
    }
  }
  catch (qi::expectation_failure<std::string::iterator> &e)
  {
    throw std::runtime_error(e.what());
  }
}

std::vector<std::vector<core::Triangle>> loadAllObjFilesInFolder(const std::string& folderPath, const bool normalize) {
    std::vector<std::vector<core::Triangle>> allTriangles;

    try {
        // Iterate over all items in the directory specified by folderPath
        for (const auto& entry : fs::directory_iterator(folderPath)) {
            // Check if the entry is a file and ends with ".obj"
            if (entry.is_regular_file() && entry.path().extension() == ".obj") {
                std::string filename = entry.path().string();
                std::vector<core::Triangle> triangles = utils::Obj::read(filename, normalize);
                allTriangles.push_back(triangles);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error processing OBJ files: " << e.what() << std::endl;
    }

    return allTriangles;
}

// Write simple obj file containing a grid of cubes, useful for testing/debugging
void write_test_cubes(const std::string &filename)
{
  static const std::array<glm::vec3, 8> UNIT_CUBE_VERTICES = {
    glm::vec3{ 1.000000, -1.000000, -1.000000 },
    glm::vec3{ 1.000000, -1.000000, 1.000000 },
    glm::vec3{ -1.000000, -1.000000, 1.000000 },
    glm::vec3{ -1.000000, -1.000000, -1.000000 },
    glm::vec3{ 1.000000, 1.000000, -1.000000 },
    glm::vec3{ 1.000000, 1.000000, 1.000000 },
    glm::vec3{ -1.000000, 1.000000, 1.000000 },
    glm::vec3{ -1.000000, 1.000000, -1.000000 },
  };

  static const std::array<std::array<int, 3>, 12> UNIT_CUBE_FACES = { {
    { { 2, 3, 4 } },
    { { 8, 7, 6 } },
    { { 5, 6, 2 } },
    { { 6, 7, 3 } },
    { { 3, 7, 8 } },
    { { 1, 4, 8 } },
    { { 1, 2, 4 } },
    { { 5, 8, 6 } },
    { { 1, 5, 2 } },
    { { 2, 6, 3 } },
    { { 4, 3, 8 } },
    { { 5, 1, 8 } },
  } };

  std::ofstream out(filename);

  if (!out.is_open())
  {
    throw std::runtime_error(fmt::format("Failed to open file '{0}' for writing"));
  }

  int n = 0;

  for (int i = -1; i <= +1; i++)
  {
    for (int j = -1; j <= +1; j++)
    {
      for (int k = -1; k <= +1; k++)
      {

        out << fmt::format("usemtl m{0}\n", n);

        for (const auto &v : UNIT_CUBE_VERTICES)
        {
          out << fmt::format("v {0} {1} {2}\n", v.x + j*4.0f, v.y + i*4.0f, v.z + k*4.0f);
        }

        for (const auto &f : UNIT_CUBE_FACES)
        {
          out << fmt::format("f {0} {1} {2}\n", f[0] + n*8, f[1] + n*8, f[2] + n*8);
        }

        n++;
      }
    }
  }

  out.close();
}

}

BOOST_FUSION_ADAPT_STRUCT(utils::Obj::Vertex, (float, x) (float, y) (float, z))
BOOST_FUSION_ADAPT_STRUCT(utils::Obj::Material, (std::string, name))

