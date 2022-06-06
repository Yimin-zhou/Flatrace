#pragma once

#include <array>
#include <limits>
#include <cmath>
#include <iostream>

#include <fmt/format.h>

namespace core {

struct Vec3
{
  static constexpr auto INF = std::numeric_limits<float>::infinity();

  float x;
  float y;
  float z;

  Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
  Vec3(const float x, const float y, const float z) : x(x), y(y), z(z) {}

  float lengthSquared() const { return x*x + y*y + z*z; }
  float length() const { return std::sqrt(lengthSquared()); }

  Vec3 normalized() const
  {
    const float l = length();

    return (l > 0.0f ? Vec3{ x / l, y / l, z / l } : Vec3());
  }

  Vec3 operator-(const Vec3 &other) const { return { x - other.x, y - other.y, z - other.z }; }
  Vec3 operator+(const Vec3 &other) const { return { x + other.x, y + other.y, z + other.z }; }
  Vec3 operator*(const float s) const { return { x * s, y * s, z * s }; }

  Vec3 cross(const Vec3 &other) const { return { y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x }; };
  float dot(const Vec3 &other) const { return x * other.x + y * other.y + z * other.z; };

  Vec3 min(const Vec3 &other) const { return { std::min(x, other.x), std::min(y, other.y), std::min(z, other.z) }; }
  Vec3 max(const Vec3 &other) const { return { std::max(x, other.x), std::max(y, other.y), std::max(z, other.z) }; }

};

struct Ray
{
  Ray(const Vec3 &origin, const Vec3 &direction)
  :
    o(origin),  d(direction), t(std::numeric_limits<float>::infinity()), dot(0.0f)
  {
  }

  Vec3 o;
  Vec3 d;

  float t;
  float dot;
};

struct Triangle
{
  Triangle() = default;
  Triangle(const Vec3 &v0, const Vec3 &v1, const Vec3 &v2)
  :
    vertices({ v0, v1, v2 }),
    edges({ v1 - v0, v2 - v0 }),
    normal(edges[0].cross(edges[1]).normalized())
  {
  }

  std::array<Vec3, 3> vertices;
  std::array<Vec3, 2> edges;
  Vec3 normal;
};

struct Plane
{
  float a;
  float b;
  float c;
  float d;

  Plane(const Vec3 &p, const Vec3 &n)
  {
    const Vec3 normal = n.normalized();

    a = normal.x;
    b = normal.y;
    c = normal.z;
    d = -(a*p.x + b*p.y + c*p.z);
  }

  float distance(const Vec3 &v) const
  {
    return (a*v.x + b*v.y + c*v.z + d);
  }
};

struct BoundingBox
{
  BoundingBox()
  :
    min({ Vec3::INF, Vec3::INF, Vec3::INF }),
    max({ -Vec3::INF, -Vec3::INF, -Vec3::INF })
  {
  }

  BoundingBox(const Vec3 &min, const Vec3 &max)
  :
    min(min), max(max)
  {
  }


  Vec3 min;
  Vec3 max;
};

}

inline std::ostream &operator<<(std::ostream &out, const core::Vec3 &v)
{
  out << fmt::format("[{0}, {1}, {2}]", v.x, v.y, v.z);
  return out;
}

inline std::ostream &operator<<(std::ostream &out, const core::Plane &p)
{
  out << fmt::format("<{0}, {1}, {2}, {3}>", p.a, p.b, p.c, p.d);
  return out;
}
inline std::ostream &operator<<(std::ostream &out, const core::BoundingBox &b)
{
  out << b.min << " - " << b.max;
  return out;
}

