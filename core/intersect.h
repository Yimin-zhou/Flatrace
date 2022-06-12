#pragma once

#include "types.h"

#include <optional>
#include <limits>

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

inline uint8_t intersect2x2(const Triangle &triangle, Ray &ray0, Ray &ray1, Ray &ray2, Ray &ray3)
{
  const bool intersect_0 = intersect(triangle, ray0);
  const bool intersect_1 = intersect(triangle, ray1);
  const bool intersect_2 = intersect(triangle, ray2);
  const bool intersect_3 = intersect(triangle, ray3);

  return intersect_0 | (intersect_1 << 1) | (intersect_2 << 2) | (intersect_3 << 3);
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

inline uint8_t intersect2x2(const BoundingBox &bbox, const Ray &ray0, const Ray &ray1, const Ray &ray2, const Ray &ray3)
{
  const bool intersect_0 = (intersect(bbox, ray0) != INF);
  const bool intersect_1 = (intersect(bbox, ray1) != INF);
  const bool intersect_2 = (intersect(bbox, ray2) != INF);
  const bool intersect_3 = (intersect(bbox, ray3) != INF);

  return intersect_0 | (intersect_1 << 1) | (intersect_2 << 2) | (intersect_3 << 3);
}

}
