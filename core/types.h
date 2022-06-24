#pragma once

#include <array>
#include <limits>
#include <cmath>
#include <iostream>

#include <fmt/format.h>

#include <simde/x86/avx2.h>

namespace core {

static constexpr float INF = std::numeric_limits<float>::infinity();
static constexpr float EPS = 1e-12f;


struct __attribute__((aligned(16))) Vec3
{
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
  Vec3 operator/(const float s) const { return { x / s, y / s, z / s }; }

  Vec3 cross(const Vec3 &other) const { return { y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x }; };
  float dot(const Vec3 &other) const { return x * other.x + y * other.y + z * other.z; };

  static Vec3 min(const Vec3 &lhs, const Vec3 &rhs) { return { std::min(lhs.x, rhs.x), std::min(lhs.y, rhs.y), std::min(lhs.z, rhs.z) }; }
  static Vec3 max(const Vec3 &lhs, const Vec3 &rhs) { return { std::max(lhs.x, rhs.x), std::max(lhs.y, rhs.y), std::max(lhs.z, rhs.z) }; }
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

  Vec3 pointOnPlane() const
  {
    if (a != 0.0f)
    {
      return { d/a, 0.0f, 0.0f };
    }
    else if (b != 0.0f)
    {
      return { 0.0f, d/b, 0.0f };
    }
    else
    {
      return { 0.0f, 0.0f, d/c };
    }
  }

  Vec3 normal() const
  {
    return { a, b, c };
  }

  float distance(const Vec3 &v) const
  {
    return (a*v.x + b*v.y + c*v.z + d);
  }

  Vec3 project(const Vec3 &v) const
  {
    return v - (normal() * distance(v));
  }
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

struct BoundingBox
{
  BoundingBox()
  :
    min({ INF, INF, INF }),
    max({ -INF, -INF, -INF })
  {
  }

  BoundingBox(const Vec3 &min, const Vec3 &max)
  :
    min(min), max(max)
  {
  }

  BoundingBox extended(const Vec3 &v) const
  {
    return { Vec3::min(min, v), Vec3::max(max, v) };
  }

  BoundingBox extended(const BoundingBox &b) const
  {
    return { Vec3::min(min, b.min), Vec3::max(max, b.max) };
  }


  BoundingBox extended(const Triangle &t) const
  {
    Vec3 min_v = min;
    Vec3 max_v = max;

    for (const Vec3 &v : t.vertices)
    {
      min_v = Vec3::min(min_v, v);
      max_v = Vec3::max(max_v, v);
    }

    return { min_v, max_v };
  }

  const float area() const
  {
    const Vec3 size = (max - min);

    const float area = 2.0f * (size.x*size.y + size.y*size.z + size.z*size.x);

    if (!std::isinf(area))
    {
      return area;
    }
    else
    {
      return (size.dot({ 1.0f, 1.0f, 1.0f }) == -INF ? 0.0f : INF);
    }
  }

  Vec3 min;
  Vec3 max;
};

struct Camera
{
  Camera(const Vec3 &p, const Vec3 &d, const Vec3 &up, const float zoom)
  :
  p(p), d(d.normalized()), zoom(zoom)
  {
    const Plane view_plane = { p, d };

    y = (view_plane.project(up) - p).normalized();
    x = view_plane.normal().cross(y);
  }

  Vec3 p;
  Vec3 d;

  Vec3 x;
  Vec3 y;

  float zoom;
};

struct Ray
{
  Ray(const Vec3 &origin, const Vec3 &direction)
  :
    o(origin),  d(direction), rd({ 1.0f / d.x, 1.0f / d.y, 1.0f / d.z }), t(INF), dot(0.0f)
  {
  }

  Vec3 o;
  Vec3 d;
  Vec3 rd;

  float t;
  float dot;
};

// 4x4 ray bundle for 8-way SIMD BVH traversal & triangle intersection
struct  __attribute__((aligned(16))) Ray4x4
{
  Ray4x4(const Camera &camera, const Vec3 &origin, const Vec3 &direction, const float DX, const float DY)
  :
    d(direction),
    rd(1.0f / direction.x, 1.0f / direction.y, 1.0f / direction.z)
  {
    alignas(32) std::array<float, 16> ox;
    alignas(32) std::array<float, 16> oy;
    alignas(32) std::array<float, 16> oz;

    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        const float x = j*DX;
        const float y = i*DY;

        const Vec3 xyz = origin + camera.x*x + camera.y*y;

        ox[i*4 + j] = xyz.x;
        oy[i*4 + j] = xyz.y;
        oz[i*4 + j] = xyz.z;
      }
    }

    _mm256_store_ps(ox_x8.data(), _mm256_load_ps(ox.data()));
    _mm256_store_ps(ox_x8.data() + 8, _mm256_load_ps(ox.data() + 8));
    _mm256_store_ps(oy_x8.data(), _mm256_load_ps(oy.data()));
    _mm256_store_ps(oy_x8.data() + 8, _mm256_load_ps(oy.data() + 8));
    _mm256_store_ps(oz_x8.data(), _mm256_load_ps(oz.data()));
    _mm256_store_ps(oz_x8.data() + 8, _mm256_load_ps(oz.data() + 8));

    std::fill(t.begin(), t.end(), INF);
    std::fill(dot.begin(), dot.end(), 0.0f);
  }

  alignas(32) std::array<float, 16> ox_x8;
  alignas(32) std::array<float, 16> oy_x8;
  alignas(32) std::array<float, 16> oz_x8;

  Vec3 d;
  Vec3 rd;

  alignas(32) std::array<float, 16> t;
  alignas(32) std::array<float, 16> dot;
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

