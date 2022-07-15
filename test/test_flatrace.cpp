#include <gtest/gtest.h>

#include "core/types.h"
#include "core/bvh.h"

#include "utils/obj.h"

using namespace core;

namespace test {

TEST(FlatRace, testConstruction)
{
  const std::vector<std::pair<int, Vec3>> POINTS = {
    { 0, { -20.0f, -20.0f, 0.0f } },
    { 1, {  11.0f,  21.0f, 0.0f } },
    { 2, { -20.0f, -10.0f, 0.0f } },
    { 3, {  21.0f,  11.0f, 0.0f } },
    { 4, {  12.0f,  -12.0f, 42.0f } },
    { 5, { -10.0f, -20.0f, 0.0f } },
  };

  std::vector<Triangle> triangles(POINTS.size());
  std::transform(POINTS.begin(), POINTS.end(), triangles.begin(), [](const auto &v) -> Triangle { return { v.first, v.second, v.second, v.second, 0 }; });

  BVH bvh(triangles);
}

TEST(FlatRace, testWriteCubes)
{
  utils::Obj::write_test_cubes("test_cubes.obj");
}

}