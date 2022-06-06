#include <gtest/gtest.h>

#include "core/types.h"
#include "core/bvh.h"

using namespace core;

namespace test {

TEST(BvhTest, testConstruction)
{
  const std::vector<Vec3> POINTS = {
    { -20.0f, -20.0f, 0.0f },
    {  11.0f,  21.0f, 0.0f },
    { -20.0f, -10.0f, 0.0f },
    {  21.0f,  11.0f, 0.0f },
    {  12.0f,  -12.0f, 42.0f },
    { -10.0f, -20.0f, 0.0f },
  };

  std::vector<Triangle> triangles(POINTS.size());
  std::transform(POINTS.begin(), POINTS.end(), triangles.begin(), [](const Vec3 &v) -> Triangle { return { v, v, v }; });

  BVH bvh(triangles);
}

}