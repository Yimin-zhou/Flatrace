#pragma once

#include <array>
#include <limits>
#include <cmath>
#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/vector_relational.hpp>

#include "third_party/fmt/format.h"

#ifdef IS_X86
  #include <immintrin.h>
#else
  #include <simde/x86/avx2.h>
#endif

namespace core {

static constexpr float INF = std::numeric_limits<float>::infinity();
static constexpr float EPS = 1e-12f;

struct Plane
{
  float a;
  float b;
  float c;
  float d;

  Plane(const glm::vec3 &p, const glm::vec3 &n)
  {
    const glm::vec3 normal = glm::normalize(n);

    a = normal.x;
    b = normal.y;
    c = normal.z;
    d = -(a*p.x + b*p.y + c*p.z);
  }

  glm::vec3 pointOnPlane() const
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

  glm::vec3 normal() const
  {
    return { a, b, c };
  }

  float distance(const glm::vec3 &v) const
  {
    return (a*v.x + b*v.y + c*v.z + d);
  }

  glm::vec3 project(const glm::vec3 &v) const
  {
    return v - (normal() * distance(v));
  }
};

struct Triangle
{
  Triangle() = default;
  Triangle(const int id, const glm::vec3 &v0, const glm::vec3 &v1, const glm::vec3 &v2, const int material)
  :
    id(id),
    vertices({ v0, v1, v2 }),
    edges({ v1 - v0, v2 - v0 }),
    normal(glm::normalize(glm::cross(edges[0], edges[1]))),
    material(material)
  {
  }

  int id;
  std::array<glm::vec3, 3> vertices;
  std::array<glm::vec3, 2> edges;
  glm::vec3 normal;
  int material;
};

struct BoundingBox
{
  BoundingBox()
  :
    min({ INF, INF, INF }),
    max({ -INF, -INF, -INF })
  {
  }

  BoundingBox(const glm::vec3 &min, const glm::vec3 &max)
  :
    min(min), max(max)
  {
  }

  BoundingBox extended(const glm::vec3 &v) const
  {
    return { glm::min(min, v), glm::max(max, v) };
  }

  BoundingBox extended(const BoundingBox &b) const
  {
    return { glm::min(min, b.min), glm::max(max, b.max) };
  }

  BoundingBox extended(const Triangle &t) const
  {
    glm::vec3 min_v = min;
    glm::vec3 max_v = max;

    for (const glm::vec3 &v : t.vertices)
    {
      min_v = glm::min(min_v, v);
      max_v = glm::max(max_v, v);
    }

    return { min_v, max_v };
  }

  const float area() const
  {
    const glm::vec3 size = (max - min);

    const float area = 2.0f * (size.x*size.y + size.y*size.z + size.z*size.x);

    if (!std::isinf(area))
    {
      return area;
    }
    else
    {
      return (glm::dot(size, { 1.0f, 1.0f, 1.0f }) == -INF ? 0.0f : INF);
    }
  }

  glm::vec3 min;
  glm::vec3 max;
};

struct Camera
{
  Camera(const glm::vec3 &p, const glm::vec3 &d, const glm::vec3 &up, const float zoom)
  :
  pos(p), dir(glm::normalize(d)), zoom(zoom) {
      const Plane view_plane = {p, d};

      y = (glm::normalize(view_plane.project(up) - p));
      x = glm::cross(view_plane.normal(), y);
  }

  glm::vec3 pos;
  glm::vec3 dir;

  glm::vec3 x;
  glm::vec3 y;

  float zoom;
};

struct Ray
{
  Ray(const glm::vec3 &origin, const glm::vec3 &direction)
  :
    o(origin),  d(direction), rd({ 1.0f / direction.x, 1.0f / direction.y, 1.0f / direction.z }),
    bvh_nodes_visited(0)
  {
  }

  void nextIntersection()
  {
    t0 = t[n];
    n++;
  }

  glm::vec3 o;
  glm::vec3 d;
  glm::vec3 rd;

  int n = 0;
  float t0 = -INF;

  std::array<float, 3> t = { INF, INF, INF };
  std::array<float, 3> dot = { 0.0f, 0.0f, 0.0f };
  std::array<int, 3> triangle = { -1, -1, -1 };

  int bvh_nodes_visited;
};

// 4x4 ray bundle for 8-way SIMD BVH traversal & triangle intersection
struct  __attribute__((aligned(16))) Ray4x4
{
  Ray4x4(const Camera &camera, const glm::vec3 &o, const glm::vec3 &d, const glm::vec3 &rd, const float DX, const float DY)
  :
    d(d), rd(rd), n(0)
  {
    alignas(32) std::array<float, 16> ox;
    alignas(32) std::array<float, 16> oy;
    alignas(32) std::array<float, 16> oz;

    // For debugging: Initialize the node visit counters
//    std::fill(bvh_nodes_visited.begin(), bvh_nodes_visited.end(), 0);

    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        const float x = j*DX;
        const float y = i*DY;

        const glm::vec3 xyz = o + camera.x*x + camera.y*y;

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

    _mm256_store_ps(t0.data(), _mm256_set1_ps(-INF));
    _mm256_store_ps(t0.data() + 8, _mm256_set1_ps(-INF));

    for (int n = 0; n < 3; n ++)
    {
      _mm256_store_ps(t.data() + n*16, _mm256_set1_ps(INF));
      _mm256_store_ps(t.data() + n*16 + 8, _mm256_set1_ps(INF));
    }

    for (int n = 0; n < 3; n ++)
    {
      _mm256_store_ps(dot.data() + n*16, _mm256_set1_ps(0.0f));
      _mm256_store_ps(dot.data() + n*16 + 8, _mm256_set1_ps(0.0f));
    }

    for (int n = 0; n < 3; n ++)
    {
      _mm256_store_si256(reinterpret_cast<__m256i *>(triangle.data() + n*16), _mm256_set1_epi32(0));
      _mm256_store_si256(reinterpret_cast<__m256i *>(triangle.data() + n*16 + 8), _mm256_set1_epi32(0));
    }
  }

  void nextIntersection()
  {
    // t0 = t[n];
    _mm256_store_ps(t0.data(), _mm256_load_ps(t.data() + n*16));
    _mm256_store_ps(t0.data() + 8, _mm256_load_ps(t.data() + n*16 + 8));

    n++;
  }

  alignas(32) std::array<float, 16> ox_x8;
  alignas(32) std::array<float, 16> oy_x8;
  alignas(32) std::array<float, 16> oz_x8;

  // For debugging
//  alignas(32) std::array<int, 16> bvh_nodes_visited;

  glm::vec3 d;
  glm::vec3 rd;

  int n;
  alignas(32) std::array<float, 16> t0;

  alignas(32) std::array<float, 48> t;
  alignas(32) std::array<float, 48> dot;
  alignas(32) std::array<int, 48> triangle;
};

}

inline std::ostream &operator<<(std::ostream &out, const glm::vec3 &v)
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

