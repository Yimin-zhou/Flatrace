#pragma once

#include "types.h"

#include <optional>
#include <limits>

namespace core {

inline bool intersect(const Triangle &triangle, Ray &ray)
{
  const Vec3 p = ray.d.cross(triangle.edges[1]);

  const float det = p.dot(triangle.edges[0]);

  if (det < 1e-12f)
  {
    return false;
  }

  const float inv_det = 1.0f / det;

  const Vec3 tv = ray.o - triangle.vertices[0];
  const float u = tv.dot(p) * inv_det;

  if (u < 0.0f || u > 1.0f)
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

inline bool intersect(const BoundingBox &bbox, const Ray &ray)
{
  const float tx1 = (bbox.min.x - ray.o.x) / ray.d.x;
  const float tx2 = (bbox.max.x - ray.o.x) / ray.d.x;
  float tmin = std::min(tx1, tx2);
  float tmax = std::max(tx1, tx2);

  const float ty1 = (bbox.min.y - ray.o.y) / ray.d.y;
  const float ty2 = (bbox.max.y - ray.o.y) / ray.d.y;
  tmin = std::max(tmin, std::min(ty1, ty2)) ;
  tmax = std::min(tmax, std::max(ty1, ty2));

  const float tz1 = (bbox.min.z - ray.o.z) / ray.d.z;
  const float tz2 = (bbox.max.z - ray.o.z) / ray.d.z;
  tmin = std::max(tmin, std::min(tz1, tz2));
  tmax = std::min(tmax, std::max(tz1, tz2));

  return ((tmax >= tmin) && (tmax > 0.0f) && (tmin < ray.t));
}

}
