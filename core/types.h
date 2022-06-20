#pragma once

#include <array>
#include <limits>
#include <cmath>
#include <iostream>

#include <fmt/format.h>

#include <simde/x86/avx2.h>

namespace core {

static constexpr float INF = std::numeric_limits<float>::max();
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

// 2x2 ray bundle for 4-way SIMD BVH traversal & triangle intersection
struct  __attribute__((aligned(16))) Ray2x2
{
  Ray2x2(const Vec3 &origin, const Vec3 &direction, const float DX, const float DY)
  {
    ox = { origin.x, origin.x + DX, origin.x, origin.x + DX };
    oy = { origin.y, origin.y, origin.y + DY, origin.y + DY };
    oz = { origin.z, origin.z, origin.z, origin.z };

    dx = { direction.x, direction.x, direction.x, direction.x };
    dy = { direction.y, direction.y, direction.y, direction.y };
    dz = { direction.z, direction.z, direction.z, direction.z };

    rdx = { 1.0f / direction.x, 1.0f / direction.x, 1.0f / direction.x, 1.0f / direction.x };
    rdy = { 1.0f / direction.y, 1.0f / direction.y, 1.0f / direction.y, 1.0f / direction.y };
    rdz = { 1.0f / direction.z, 1.0f / direction.z, 1.0f / direction.z, 1.0f / direction.z };

    t = { INF, INF, INF, INF };
    dot = { 0.0f, 0.0f, 0.0f, 0.0f };
  }

  std::array<float, 4> ox;
  std::array<float, 4> oy;
  std::array<float, 4> oz;

  std::array<float, 4> dx;
  std::array<float, 4> dy;
  std::array<float, 4> dz;

  std::array<float, 4> rdx;
  std::array<float, 4> rdy;
  std::array<float, 4> rdz;

  std::array<float, 4> t;
  std::array<float, 4> dot;
};

// 4x4 ray bundle for 8-way SIMD BVH traversal & triangle intersection
struct  __attribute__((aligned(16))) Ray4x4
{
  Ray4x4(const Vec3 &origin, const Vec3 &direction, const float DX, const float DY)
  :
    d(direction),
    rd(1.0f / direction.x, 1.0f / direction.y, 1.0f / direction.z)
  {
    std::array<float, 16> ox;
    std::array<float, 16> oy;

    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        ox[i*4 + j] = origin.x + j*DX;
        oy[i*4 + j] = origin.y + i*DY;
      }
    }

    _mm256_store_ps(ox_x8.data(), _mm256_load_ps(ox.data()));
    _mm256_store_ps(ox_x8.data() + 8, _mm256_load_ps(ox.data() + 8));
    _mm256_store_ps(oy_x8.data(), _mm256_load_ps(oy.data()));
    _mm256_store_ps(oy_x8.data() + 8, _mm256_load_ps(oy.data() + 8));

    oz = origin.z;

    std::fill(t.begin(), t.end(), INF);
    std::fill(dot.begin(), dot.end(), 0.0f);
  }

  alignas(32) std::array<float, 16> ox_x8;
  alignas(32) std::array<float, 16> oy_x8;
  float oz;

  Vec3 d;
  Vec3 rd;

  alignas(32) std::array<float, 16> t;
  alignas(32) std::array<float, 16> dot;
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
    min({ INF, INF, INF }),
    max({ -INF, -INF, -INF })
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

