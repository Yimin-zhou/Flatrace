// Worlds' stupidest .OBJ parser

#include "obj.h"

#include <boost/spirit/include/qi.hpp>
#include <boost/fusion/include/adapt_struct.hpp>

#include <variant>
#include <fstream>

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

struct Group { std::string name; };
struct Material { std::string name; };

using Face = std::vector<int>;
using Statement = std::variant<Vertex, Face, Group, Material>;

struct ObjLineParser : public qi::grammar<std::string::const_iterator, std::vector<Statement>(), qi::blank_type>
{
  template<typename T = void>
  using Rule = qi::rule<iterator_type, T(), qi::blank_type>;

  ObjLineParser() : ObjLineParser::base_type(obj)
  {
    label = *(qi::char_ - qi::eol);

    vertex = qi::lexeme['v'] >> (qi::double_ > qi::double_ > qi::double_ > qi::eol);
    vertexIndex = (qi::int_ >> qi::omit[*('/' >> -qi::int_)]);
    face = qi::lexeme['f'] >> (qi::repeat(3, 4)[vertexIndex] > qi::eol);

    group = qi::lexeme['g'] >> (label > qi::eol);
    material = qi::lexeme["usemtl"] >> (label > qi::eol);

    unknown = *(qi::char_ - qi::eol) >> qi::eol;

    line = (vertex | face | group | material | qi::omit[unknown]);

    obj = *line >> qi::eoi;
  }

  Rule<std::string> label;
  Rule<Vertex> vertex;
  Rule<int> vertexIndex;
  Rule<Face> face;
  Rule<Group> group;
  Rule<Material> material;
  Rule<> unknown;
  Rule<Statement> line;
  Rule<std::vector<Statement>> obj;
};

// Visitor class that interprets parsed OBJ statements, to turn them into triangle soup
class ObjInterpreter
{
  public:
  ObjInterpreter()
  :
  _firstGroupVertex(0)
  {
  }

  void operator()(const Obj::Vertex &vertex)
  {
    _vertices.push_back({ vertex.x, vertex.y, vertex.z });
  }

  void operator()(const Obj::Face &face)
  {
    if (face.size() == 3)
    {
      _triangles.push_back({ _vertices[_firstGroupVertex + face[0] - 1], _vertices[_firstGroupVertex + face[1] - 1], _vertices[_firstGroupVertex + face[2] -1] });
    }
    else
    {
      _triangles.push_back({ _vertices[_firstGroupVertex + face[0] - 1], _vertices[_firstGroupVertex + face[1] - 1], _vertices[_firstGroupVertex + face[2] - 1] });
      _triangles.push_back({ _vertices[_firstGroupVertex + face[0] - 1], _vertices[_firstGroupVertex + face[2] - 1], _vertices[_firstGroupVertex + face[3] - 1] });
    }
  }

  void operator()(const Obj::Group &group)
  {
    // We don't care about any grouping, but we do need to 'reset' the vertex numbering any time
    // we encounter a group statement, as all subsequent statements will restart their vertex numbering at one
    _firstGroupVertex = _vertices.size();
  }

  void operator()(const Obj::Material &material)
  {
    /// \todo ignore materials for now
  }

  const std::vector<core::Triangle> &getTriangles() const { return _triangles; }

  private:
  int _firstGroupVertex;

  std::vector<core::Vec3> _vertices;
  std::vector<core::Triangle> _triangles;
};

// Read .OBJ file contents and return its contents as triangle soup
std::vector<core::Triangle> read(const std::string &filename)
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

      return obj_interpreter.getTriangles();
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
BOOST_FUSION_ADAPT_STRUCT(utils::Obj::Group, (std::string, name))
BOOST_FUSION_ADAPT_STRUCT(utils::Obj::Material, (std::string, name))

