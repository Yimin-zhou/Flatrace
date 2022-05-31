#pragma once

#include <array>

namespace core {

struct Vec3
{
  float x;
  float y;
  float z;

  Vec3 operator-(const Vec3 &other) const
  {
    return { x - other.x, y - other.y, z - other.z };
  }
};

struct Ray
{
  Vec3 o;
  Vec3 d;
};

struct Triangle
{
  Triangle(const Vec3 &v0, const Vec3 &v1, const Vec3 &v2)
  :
    vertices({ v0, v1, v2 }),
    edges({ v1 - v0, v2 - v1, v0 - v2 })
  {
  }

  std::array<Vec3, 3> vertices;
  std::array<Vec3, 3> edges;
};

}
