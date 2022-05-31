#pragma once

#include "types.h"
#include "math.h"

namespace core {

bool intersect(const Triangle &triangle, const Ray &ray)
{
  const Vec3 p = cross(ray.d, triangle.edges[1]);

  const float det = dot(p, triangle.edges[0]);

  if (det < 1e-12f)
  {
    return false;
  }

  const Vec3 t = ray.o - triangle.vertices[0];

  const float u = dot(p, t);

  if (u < 0.0f || u > det)
  {
    return false;
  }

  float v = dot(cross(t, triangle.edges[0]), ray.d);

  if (v < 0.0f || v > det)
  {
    return false;
  }

  return ((det - u - v) >= 0.0f);
}

}
