#pragma once

#include "types.h"

#include <optional>
#include <limits>

#include <simde/x86/sse4.2.h>

#include <fmt/format.h>

namespace {

struct Vec3_x4
{
  __m128 x;
  __m128 y;
  __m128 z;
};

// 4-way vector dot product
__m128 dot4(const Vec3_x4 &a, const Vec3_x4 &b)
{
  const __m128 px = _mm_mul_ps(a.x, b.x);
  const __m128 py = _mm_mul_ps(a.y, b.y);
  const __m128 pz = _mm_mul_ps(a.z, b.z);

  return _mm_add_ps(_mm_add_ps(px, py), pz);
}

// 4-way vector cross product
Vec3_x4 cross4(const Vec3_x4 &a, const Vec3_x4 &b)
{
  return {
    _mm_sub_ps(_mm_mul_ps(a.y, b.z), _mm_mul_ps(a.z, b.y)),
    _mm_sub_ps(_mm_mul_ps(a.z, b.x), _mm_mul_ps(a.x, b.z)),
    _mm_sub_ps(_mm_mul_ps(a.x, b.y), _mm_mul_ps(a.y, b.x))
  };
}

}

namespace core {

inline bool intersect(const Triangle &triangle, Ray &ray)
{
  const Vec3 p = ray.d.cross(triangle.edges[1]);

  const float det = p.dot(triangle.edges[0]);

  if (det < EPS)
  {
    return false;
  }

  const float inv_det = 1.0f / det;

  const Vec3 tv = ray.o - triangle.vertices[0];
  const float u = tv.dot(p) * inv_det;

  if ((u < 0.0f) || (u > 1.0f))
  {
    return false;
  }

  const Vec3 qv = tv.cross(triangle.edges[0]);
  const float v = qv.dot(ray.d) * inv_det;

  if ((v < 0.0f) || ((u + v) > 1.0f))
  {
    return false;
  }

  const float t = qv.dot(triangle.edges[1]) * inv_det;

  if (t < ray.t)
  {
    ray.t = t;
    ray.dot = triangle.normal.dot(ray.d);
  }

  return true;
}

inline void intersect2x2(const Triangle &triangle, Ray2x2 &rays)
{
  constexpr int NONE = 0b0000;

  static const __m128 zero = _mm_set1_ps(0.0f);
  static const __m128 one = _mm_set1_ps(1.0f);

  // const Vec3 p = ray.d.cross(triangle.edges[1]);
  const Vec3_x4 ray_d = {
    _mm_load_ps(rays.dx.data()),
    _mm_load_ps(rays.dy.data()),
    _mm_load_ps(rays.dz.data())
  };

  const Vec3_x4 triangle_e0 = {
    _mm_load1_ps(&triangle.edges[0].x),
    _mm_load1_ps(&triangle.edges[0].y),
    _mm_load1_ps(&triangle.edges[0].z)
  };

  const Vec3_x4 triangle_e1 = {
    _mm_load1_ps(&triangle.edges[1].x),
    _mm_load1_ps(&triangle.edges[1].y),
    _mm_load1_ps(&triangle.edges[1].z)
  };

  const Vec3_x4 p = cross4(ray_d, triangle_e1);

  //  const float det = p.dot(triangle.edges[0]);
  //
  //  if (det < EPS)
  //  {
  //    return false;
  //  }
  const __m128 det = dot4(p, triangle_e0);

  __m128 update_rays = _mm_cmpgt_ps(det, _mm_set1_ps(EPS));

  if (_mm_movemask_ps(update_rays) == NONE)
  {
    return;
  }

  // const float inv_det = 1.0f / det;
  const __m128 inv_det = _mm_div_ps(_mm_set1_ps(1.0f), det);

  //  const Vec3 tv = ray.o - triangle.vertices[0];
  //  const float u = tv.dot(p) * inv_det;
  //
  //  if ((u < 0.0f) || (u > 1.0f))
  //  {
  //    return false;
  //  }
  const Vec3_x4 tv = {
    _mm_sub_ps(_mm_load_ps(rays.ox.data()), _mm_load1_ps(&triangle.vertices[0].x)),
    _mm_sub_ps(_mm_load_ps(rays.oy.data()), _mm_load1_ps(&triangle.vertices[0].y)),
    _mm_sub_ps(_mm_load_ps(rays.oz.data()), _mm_load1_ps(&triangle.vertices[0].z)),
  };

  const __m128 u = _mm_mul_ps(dot4(tv, p), inv_det);

  update_rays = _mm_and_ps(update_rays, _mm_cmpge_ps(u, zero));
  update_rays = _mm_and_ps(update_rays, _mm_cmple_ps(u, one));

  if (_mm_movemask_ps(update_rays) == NONE)
  {
    return;
  }

  //  const Vec3 qv = tv.cross(triangle.edges[0]);
  //  const float v = qv.dot(ray.d) * inv_det;
  //
  //  if ((v < 0.0f) || ((u + v) > 1.0f))
  //  {
  //    return false;
  //  }

  const Vec3_x4 qv = cross4(tv, triangle_e0);
  const __m128 v = _mm_mul_ps(dot4(qv, ray_d), inv_det);
  const __m128 u_plus_v = _mm_add_ps(u, v);

  update_rays = _mm_and_ps(update_rays, _mm_cmpge_ps(v, zero));
  update_rays = _mm_and_ps(update_rays, _mm_cmple_ps(u_plus_v, one));

  if (_mm_movemask_ps(update_rays) == NONE)
  {
    return;
  }

  //  const float t = qv.dot(triangle.edges[1]) * inv_det;
  const __m128 qv_dot_e1 = dot4(qv, triangle_e1);
  const __m128 t = _mm_mul_ps(qv_dot_e1, inv_det);

  const __m128 ray_t =  _mm_load_ps(rays.t.data());
  const __m128 ray_dot =  _mm_load_ps(rays.dot.data());

  //  if (t < ray.t)
  //  {
  //    ray.t = t;
  //    ray.dot = triangle.normal.dot(ray.d);
  //  }
  update_rays = _mm_and_ps(update_rays, _mm_cmplt_ps(t, ray_t));

  const int t_mask = _mm_movemask_ps(update_rays);

  if (t_mask != NONE)
  {
    const Vec3_x4 triangle_normal = {
      _mm_load1_ps(&triangle.normal.x),
      _mm_load1_ps(&triangle.normal.y),
      _mm_load1_ps(&triangle.normal.z),
    };

    const __m128 dot = dot4(triangle_normal, ray_d);

    const __m128 new_t = _mm_blend_ps(ray_t, t, t_mask);
    const __m128 new_dot = _mm_blend_ps(ray_dot, dot, t_mask);

    _mm_store_ps(rays.t.data(), new_t);
    _mm_store_ps(rays.dot.data(), new_dot);
  }
}

inline float intersect(const BoundingBox &bbox, const Ray &ray)
{
  const float tx1 = (bbox.min.x - ray.o.x) * ray.rd.x;
  const float tx2 = (bbox.max.x - ray.o.x) * ray.rd.x;
  float tmin = std::min(tx1, tx2);
  float tmax = std::max(tx1, tx2);

  const float ty1 = (bbox.min.y - ray.o.y) * ray.rd.y;
  const float ty2 = (bbox.max.y - ray.o.y) * ray.rd.y;
  tmin = std::max(tmin, std::min(ty1, ty2)) ;
  tmax = std::min(tmax, std::max(ty1, ty2));

  const float tz1 = (bbox.min.z - ray.o.z) * ray.rd.z;
  const float tz2 = (bbox.max.z - ray.o.z) * ray.rd.z;
  tmin = std::max(tmin, std::min(tz1, tz2));
  tmax = std::min(tmax, std::max(tz1, tz2));

  const bool hit = ((tmax >= tmin) && (tmax > 0.0f) && (tmin < ray.t));

  return (hit ? tmin : INF);
}

inline uint8_t intersect2x2(const BoundingBox &bbox, const Ray2x2 &rays)
{
  //  const float tx1 = (bbox.min.x - ray.o.x) * ray.rd.x;
  //  const float tx2 = (bbox.max.x - ray.o.x) * ray.rd.x;
  //  float t_min = std::min(tx1, tx2);
  //  float t_max = std::max(tx1, tx2);
  const __m128 ray_o_x = _mm_load_ps(rays.ox.data());
  const __m128 ray_rd_x = _mm_load_ps(rays.rdx.data());

  const __m128 tx1 = _mm_mul_ps(_mm_sub_ps(_mm_load1_ps(&bbox.min.x), ray_o_x), ray_rd_x);
  const __m128 tx2 = _mm_mul_ps(_mm_sub_ps(_mm_load1_ps(&bbox.max.x), ray_o_x), ray_rd_x);

  __m128 t_min = _mm_min_ps(tx1, tx2);
  __m128 t_max = _mm_max_ps(tx1, tx2);

  //  const float ty1 = (bbox.min.y - ray.o.y) * ray.rd.y;
  //  const float ty2 = (bbox.max.y - ray.o.y) * ray.rd.y;
  //  t_min = std::max(tmin, std::min(ty1, ty2)) ;
  //  t_max = std::min(tmax, std::max(ty1, ty2));
  const __m128 ray_o_y = _mm_load_ps(rays.oy.data());
  const __m128 ray_rd_y = _mm_load_ps(rays.rdy.data());

  const __m128 ty1 = _mm_mul_ps(_mm_sub_ps(_mm_load1_ps(&bbox.min.y), ray_o_y), ray_rd_y);
  const __m128 ty2 = _mm_mul_ps(_mm_sub_ps(_mm_load1_ps(&bbox.max.y), ray_o_y), ray_rd_y);

  t_min = _mm_max_ps(t_min, _mm_min_ps(ty1, ty2));
  t_max = _mm_min_ps(t_max, _mm_max_ps(ty1, ty2));

  //  const float tz1 = (bbox.min.z - ray.o.z) * ray.rd.z;
  //  const float tz2 = (bbox.max.z - ray.o.z) * ray.rd.z;
  //  t_min = std::max(tmin, std::min(tz1, tz2));
  //  t_max = std::min(tmax, std::max(tz1, tz2));
  const __m128 ray_o_z = _mm_load_ps(rays.oz.data());
  const __m128 ray_rd_z = _mm_load_ps(rays.rdz.data());

  const __m128 tz1 = _mm_mul_ps(_mm_sub_ps(_mm_load1_ps(&bbox.min.z), ray_o_z), ray_rd_z);
  const __m128 tz2 = _mm_mul_ps(_mm_sub_ps(_mm_load1_ps(&bbox.max.z), ray_o_z), ray_rd_z);

  t_min = _mm_max_ps(t_min, _mm_min_ps(tz1, tz2));
  t_max = _mm_min_ps(t_max, _mm_max_ps(tz1, tz2));

  // const bool hit = ((tmax >= tmin) && (tmax > 0.0f) && (tmin < ray.t));
  const __m128 zero = _mm_set1_ps(0.0f);
  const __m128 ray_t = _mm_load_ps(rays.t.data());

  const __m128 hit = _mm_and_ps(_mm_and_ps(_mm_cmpge_ps(t_max, t_min), _mm_cmpgt_ps(t_max, zero)), _mm_cmplt_ps(t_min, ray_t));

  // return (hit[0] | (hit[1] << 1) | (hit[2] << 2) | (hit[3] << 3);
  return _mm_movemask_ps(hit);
}

}
