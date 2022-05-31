#pragma once

#include "types.h"

namespace core {

Vec3 cross(const Vec3 &v, const Vec3 &w)
{
  return {v.y * w.z - v.z * w.y, v.z * w.x - v.x * w.z, v.x * w.y - v.y * w.x};
}

float dot(const Vec3 &v, const Vec3 &w)
{
  return v.x * w.x + v.y * w.y + v.z * w.z;
}

}
