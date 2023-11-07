#pragma once

#include "types.h"

#include <optional>
#include <limits>

#ifdef IS_X86
  #include <immintrin.h>
#else
  #include <simde/x86/avx2.h>
#endif

#include "third_party/fmt/format.h"

namespace {

struct Vec3_x4
{
  __m128 x;
  __m128 y;
  __m128 z;
};

struct Vec3_x8
{
  Vec3_x8(const glm::vec3 &v)
  :
    x(_mm256_broadcast_ss(&v.x)),
    y(_mm256_broadcast_ss(&v.y)),
    z(_mm256_broadcast_ss(&v.z))
  {
  }

  Vec3_x8(const __m256 x_x8, const __m256 y_x8, const __m256 z_x8)
  :
    x(x_x8), y(y_x8), z(z_x8)
  {
  }
  __m256 x;
  __m256 y;
  __m256 z;
};

// 4-way vector dot product
__m128 dot4(const Vec3_x4 &a, const Vec3_x4 &b)
{
  const __m128 px = _mm_mul_ps(a.x, b.x);
  const __m128 py = _mm_mul_ps(a.y, b.y);
  const __m128 pz = _mm_mul_ps(a.z, b.z);

  return _mm_add_ps(_mm_add_ps(px, py), pz);
}

// 8-way vector dot product
__m256 dot8(const Vec3_x8 &a, const Vec3_x8 &b)
{
  const __m256 px = _mm256_mul_ps(a.x, b.x);
  const __m256 py = _mm256_mul_ps(a.y, b.y);
  const __m256 pz = _mm256_mul_ps(a.z, b.z);

  return _mm256_add_ps(_mm256_add_ps(px, py), pz);
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

// 8-way vector cross product
Vec3_x8 cross8(const Vec3_x8 &a, const Vec3_x8 &b)
{
  return {
    _mm256_sub_ps(_mm256_mul_ps(a.y, b.z), _mm256_mul_ps(a.z, b.y)),
    _mm256_sub_ps(_mm256_mul_ps(a.z, b.x), _mm256_mul_ps(a.x, b.z)),
    _mm256_sub_ps(_mm256_mul_ps(a.x, b.y), _mm256_mul_ps(a.y, b.x))
  };
}

}

namespace core {

inline bool intersect(const Triangle &triangle, Ray &ray)
{
  const glm::vec3 p = glm::cross(ray.d, triangle.edges[1]);

  const float det = glm::dot(p, triangle.edges[0]);

  if (det < EPS)
  {
    return false;
  }

  const float inv_det = 1.0f / det;

  const glm::vec3 tv = ray.o - triangle.vertices[0];
  const float u = glm::dot(tv, p) * inv_det;

  if ((u < 0.0f) || (u > 1.0f))
  {
    return false;
  }

  const glm::vec3 qv = glm::cross(tv, triangle.edges[0]);
  const float v = glm::dot(qv, ray.d);

  if ((v < 0.0f) || ((u + v) > 1.0f))
  {
    return false;
  }

  const float t = glm::dot(qv, triangle.edges[1]);

  if ((t < ray.t[ray.n]) && (t > ray.t0))
  {
    ray.t[ray.n] = t;
    ray.dot[ray.n] = glm::dot(triangle.normal, ray.d);
    ray.triangle[ray.n] = triangle.id;
  }

  return true;
}

inline void intersect4x4(const Triangle &triangle, Ray4x4 &rays)
{
  const __m256 ZERO_X8 = _mm256_set1_ps(0.0f);
  const __m256 ONE_X8 = _mm256_set1_ps(1.0f);

  const Vec3_x8 triangle_e0 = {
    _mm256_broadcast_ss(&triangle.edges[0].x),
    _mm256_broadcast_ss(&triangle.edges[0].y),
    _mm256_broadcast_ss(&triangle.edges[0].z)
  };

  const Vec3_x8 triangle_e1 = {
    _mm256_broadcast_ss(&triangle.edges[1].x),
    _mm256_broadcast_ss(&triangle.edges[1].y),
    _mm256_broadcast_ss(&triangle.edges[1].z)
  };

  const Vec3_x8 ray_d = {
    _mm256_broadcast_ss(&rays.d.x),
    _mm256_broadcast_ss(&rays.d.y),
    _mm256_broadcast_ss(&rays.d.z),
  };

  // const Vec3 p = ray.d.cross(triangle.edges[1]);
  const glm::vec3 p = glm::cross(rays.d, triangle.edges[1]);

  //  const float det = p.dot(triangle.edges[0]);
  //
  //  if (det < EPS)
  //  {
  //    return false;
  //  }
  const float det = glm::dot(p, triangle.edges[0]);

  if (det < EPS)
  {
    return;
  }

  const float inv_det = 1.0f / det;

  for (int i = 0; i < 2; i++)
  {
    __m256 update_rays = _mm256_castsi256_ps(_mm256_set1_epi32(0xFFFFFFFF));

    const Vec3_x8 p_x8(p);

    const __m256 inv_det_x8 = _mm256_broadcast_ss(&inv_det);

    //  const Vec3 tv = ray.o - triangle.vertices[0];
    //  const float u = tv.dot(p) * inv_det;
    //
    //  if ((u < 0.0f) || (u > 1.0f))
    //  {
    //    return false;
    //  }
    const Vec3_x8 tv = {
      _mm256_sub_ps(_mm256_load_ps(rays.ox_x8.data() + i*8), _mm256_broadcast_ss(&triangle.vertices[0].x)),
      _mm256_sub_ps(_mm256_load_ps(rays.oy_x8.data() + i*8), _mm256_broadcast_ss(&triangle.vertices[0].y)),
      _mm256_sub_ps(_mm256_load_ps(rays.oz_x8.data() + i*8), _mm256_broadcast_ss(&triangle.vertices[0].z))
    };

    const __m256 u = _mm256_mul_ps(dot8(tv, p_x8), inv_det_x8);

    update_rays = _mm256_and_ps(update_rays, _mm256_cmp_ps(u, ZERO_X8, _CMP_GE_OQ));
    update_rays = _mm256_and_ps(update_rays, _mm256_cmp_ps(u, ONE_X8, _CMP_LE_OQ));

    if (_mm256_testz_ps(update_rays, update_rays))
    {
      continue;
    }

    //  const Vec3 qv = tv.cross(triangle.edges[0]);
    //  const float v = qv.dot(ray.d) * inv_det;
    //
    //  if ((v < 0.0f) || ((u + v) > 1.0f))
    //  {
    //    return false;
    //  }

    const Vec3_x8 qv = cross8(tv, triangle_e0);
    const __m256 v = _mm256_mul_ps(dot8(qv, ray_d), inv_det_x8);
    const __m256 u_plus_v = _mm256_add_ps(u, v);

    update_rays = _mm256_and_ps(update_rays, _mm256_cmp_ps(v, ZERO_X8, _CMP_GE_OQ));
    update_rays = _mm256_and_ps(update_rays, _mm256_cmp_ps(u_plus_v, ONE_X8, _CMP_LE_OQ));

    if (_mm256_testz_ps(update_rays, update_rays))
    {
      continue;
    }

    //  const float t = qv.dot(triangle.edges[1]) * inv_det;
    const __m256 qv_dot_e1 = dot8(qv, triangle_e1);
    const __m256 t = _mm256_mul_ps(qv_dot_e1, inv_det_x8);

    //  if ((t < ray.t[ray.n]) && (t > ray.t0))
    //  {
    //    ray.t[ray.n] = t;
    //    ray.dot[ray.n] = triangle.normal.dot(ray.d);
    //  }
    const __m256 ray_t = _mm256_load_ps(rays.t.data() + rays.n*16 + i*8);
    const __m256 ray_t0 = _mm256_load_ps(rays.t0.data() + i*8);
    const __m256 ray_dot = _mm256_load_ps(rays.dot.data() + rays.n*16 + i*8);
    const __m256i ray_triangle = _mm256_load_si256(reinterpret_cast<const __m256i *>(rays.triangle.data() + rays.n*16 + i*8));

    update_rays = _mm256_and_ps(update_rays, _mm256_cmp_ps(t, ray_t, _CMP_LT_OQ));
    update_rays = _mm256_and_ps(update_rays, _mm256_cmp_ps(t, ray_t0, _CMP_GT_OQ));

    if (!_mm256_testz_ps(update_rays, update_rays))
    {
      const __m256 dot = dot8(Vec3_x8(triangle.normal), ray_d);

      const __m256 new_t = _mm256_blendv_ps(ray_t, t, update_rays);
      const __m256 new_dot = _mm256_blendv_ps(ray_dot, dot, update_rays);
//      const __m256i new_triangle =  _mm256_set1_epi32(triangle.id);
       const __m256i new_triangle = _mm256_castps_si256(_mm256_blendv_ps(_mm256_castsi256_ps(ray_triangle), _mm256_castsi256_ps(_mm256_set1_epi32(triangle.id)), update_rays));

      _mm256_store_ps(rays.t.data() + rays.n*16 + i*8, new_t);
      _mm256_store_ps(rays.dot.data() + rays.n*16 + i*8, new_dot);
      _mm256_store_si256(reinterpret_cast<__m256i *>(rays.triangle.data() + rays.n*16 + i*8), new_triangle);
    }
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

  const bool hit = ((tmax >= tmin) && (tmax > 0.0f) && (tmin < ray.t[ray.n]) && (tmax > ray.t0));

  return (hit ? tmin : INF);
}

inline float intersect4x4(const BoundingBox &bbox, const Ray4x4 &rays)
{
  const __m256 ZERO_x8 = _mm256_set1_ps(0.0f);

  const __m256 ray_rd_x = _mm256_broadcast_ss(&rays.rd.x);
  const __m256 ray_rd_y = _mm256_broadcast_ss(&rays.rd.y);
  const __m256 ray_rd_z = _mm256_broadcast_ss(&rays.rd.z);

  for (int i = 0; i < 2; i++)
  {
    //  const float tx1 = (bbox.min.x - ray.o.x) * ray.rd.x;
    //  const float tx2 = (bbox.max.x - ray.o.x) * ray.rd.x;
    //  float t_min = std::min(tx1, tx2);
    //  float t_max = std::max(tx1, tx2);
    const __m256 tx1 = _mm256_mul_ps(_mm256_sub_ps(_mm256_broadcast_ss(&bbox.min.x), _mm256_load_ps(rays.ox_x8.data() + i*8)), ray_rd_x);
    const __m256 tx2 = _mm256_mul_ps(_mm256_sub_ps(_mm256_broadcast_ss(&bbox.max.x), _mm256_load_ps(rays.ox_x8.data() + i*8)), ray_rd_x);

    __m256 t_min = _mm256_min_ps(tx1, tx2);
    __m256 t_max = _mm256_max_ps(tx1, tx2);

    //  const float ty1 = (bbox.min.y - ray.o.y) * ray.rd.y;
    //  const float ty2 = (bbox.max.y - ray.o.y) * ray.rd.y;
    //  t_min = std::max(tmin, std::min(ty1, ty2)) ;
    //  t_max = std::min(tmax, std::max(ty1, ty2));
    const __m256 ty1 = _mm256_mul_ps(_mm256_sub_ps(_mm256_broadcast_ss(&bbox.min.y), _mm256_load_ps(rays.oy_x8.data() + i*8)), ray_rd_y);
    const __m256 ty2 = _mm256_mul_ps(_mm256_sub_ps(_mm256_broadcast_ss(&bbox.max.y), _mm256_load_ps(rays.oy_x8.data() + i*8)), ray_rd_y);

    t_min = _mm256_max_ps(t_min, _mm256_min_ps(ty1, ty2));
    t_max = _mm256_min_ps(t_max, _mm256_max_ps(ty1, ty2));

    //  const float tz1 = (bbox.min.z - ray.o.z) * ray.rd.z;
    //  const float tz2 = (bbox.max.z - ray.o.z) * ray.rd.z;
    //  t_min = std::max(tmin, std::min(tz1, tz2));
    //  t_max = std::min(tmax, std::max(tz1, tz2));
    const __m256 tz1 = _mm256_mul_ps(_mm256_sub_ps(_mm256_broadcast_ss(&bbox.min.z), _mm256_load_ps(rays.oz_x8.data() + i*8)), ray_rd_z);
    const __m256 tz2 = _mm256_mul_ps(_mm256_sub_ps(_mm256_broadcast_ss(&bbox.max.z), _mm256_load_ps(rays.oz_x8.data() + i*8)), ray_rd_z);

    t_min = _mm256_max_ps(t_min, _mm256_min_ps(tz1, tz2));
    t_max = _mm256_min_ps(t_max, _mm256_max_ps(tz1, tz2));

    // const bool hit = ((tmax >= tmin) && (tmax > 0.0f) && (tmin < ray.t) && (tmax > ray.t0))
    const __m256 ray_t = _mm256_load_ps(rays.t.data() + rays.n*16 + i*8);

    __m256 h = _mm256_castsi256_ps(_mm256_set1_epi32(0xFFFFFFFF));

    h = _mm256_cmp_ps(t_max, t_min, _CMP_GE_OQ);
    h = _mm256_and_ps(h, _mm256_cmp_ps(t_max, ZERO_x8, _CMP_GT_OQ));
    h = _mm256_and_ps(h, _mm256_cmp_ps(t_min, ray_t, _CMP_LT_OQ));
    h = _mm256_and_ps(h, _mm256_cmp_ps(t_max, _mm256_load_ps(rays.t0.data() + i*8), _CMP_GT_OQ));

    if (!_mm256_testz_si256(_mm256_castps_si256(h), _mm256_castps_si256(h)))
    {
      // Find minimum t value over all rays that hit something and return it. We use a series of
      // lane swaps (permutes) and min operations to avoid branches, which has shown to be faster
      // than copying out the 8 t-values and doing a (non-vectorized) std::min_element on them.
      const __m256 v0 = _mm256_blendv_ps(_mm256_set1_ps(INF), t_min, h);
      const __m256 v1 = _mm256_permute_ps(v0, 0b10'11'00'01);
      const __m256 v2 = _mm256_min_ps(v0, v1);
      const __m256 v3 = _mm256_permute_ps(v2, 0b01'00'11'10);
      const __m256 v4 = _mm256_min_ps(v2, v3);
      const __m256 v5 = _mm256_castpd_ps(_mm256_permute4x64_pd(_mm256_castps_pd(v4), 0b01'00'11'10));
      const __m256 t_hit_min = _mm256_min_ps(v4, v5);

      return _mm256_cvtss_f32(t_hit_min);
    }
  }

  return INF;
}



}
