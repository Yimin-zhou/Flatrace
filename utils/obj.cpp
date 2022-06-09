// Worlds' stupidest .OBJ parser

#include "obj.h"

#include <boost/spirit/include/qi.hpp>
#include <boost/fusion/include/adapt_struct.hpp>

#include <fstream>
#include <variant>
#include <array>
#include <tuple>

#include <fmt/format.h>

namespace qi = boost::spirit::qi;

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
      _triangles.push_back({ _vertices[face[0] - 1], _vertices[face[1] - 1], _vertices[face[2] - 1] });

      if (face.size() == 4)
      {
        _triangles.push_back({ _vertices[face[0] - 1], _vertices[face[2] - 1], _vertices[face[3] - 1] });
      }
    }

    void operator()(const Obj::Material &material)
    {
      /// \todo ignore materials for now
    }

    const std::vector<core::Triangle> &getTriangles() const { return _triangles; }

  private:
    std::vector<core::Vec3> _vertices;
    std::vector<core::Triangle> _triangles;
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
        core::Vec3 xyz_min = { core::INF,  core::INF, core::INF };
        core::Vec3 xyz_max = { -core::INF,  -core::INF, -core::INF };

        for (const core::Triangle &triangle : triangles)
        {
          for (const core::Vec3 &v : triangle.vertices)
          {
            xyz_min = core::Vec3::min(xyz_min, v);
            xyz_max = core::Vec3::max(xyz_max, v);
          }
        }

        const core::Vec3 center = (xyz_min + xyz_max) / 2.0f;
        const core::Vec3 range = (xyz_max - xyz_min);
        const float max_dim = std::max(range.x, std::max(range.y, range.z));

        std::transform(triangles.begin(), triangles.end(), triangles.begin(), [center, max_dim](core::Triangle &triangle)
        {
          const core::Vec3 v0 = (triangle.vertices[0] - center) / max_dim;
          const core::Vec3 v1 = (triangle.vertices[1] - center) / max_dim;
          const core::Vec3 v2 = (triangle.vertices[2] - center) / max_dim;

          return core::Triangle{ v0, v1, v2 };
        });
      }

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

}

BOOST_FUSION_ADAPT_STRUCT(utils::Obj::Vertex, (float, x) (float, y) (float, z))
BOOST_FUSION_ADAPT_STRUCT(utils::Obj::Material, (std::string, name))

