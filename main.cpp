#include "core/types.h"
#include "core/intersect.h"
#include "core/frame.h"
#include "core/bvh.h"

#include "utils/ppm.h"
#include "utils/obj.h"

#include <fmt/format.h>

#include <chrono>
#include <iostream>
#include <iomanip>

using namespace core;

int main(int argc, char **argv)
{
  constexpr auto FRAME_WIDTH = 1024;
  constexpr auto FRAME_HEIGHT = 768;

  constexpr float VIEWPORT_WIDTH = 1.2f;
  constexpr float VIEWPORT_HEIGHT = 1.2f;

  constexpr float DX = VIEWPORT_WIDTH / FRAME_WIDTH;
  constexpr float DY = VIEWPORT_HEIGHT / FRAME_HEIGHT;

  constexpr auto N_FRAMES = 1;
  constexpr auto N_RAYS = N_FRAMES * FRAME_WIDTH * FRAME_HEIGHT;

  using namespace std::chrono;

  if (argc != 2)
  {
    std::cerr << "\nUsage: flatrace <mesh_file.obj>\n\n";
    return EXIT_FAILURE;
  }

  const std::string input_file(argv[1]);

  // Load getTriangle data
  std::vector<Triangle> triangles;

  try
  {
    triangles = utils::Obj::read(input_file);
  }
  catch (std::runtime_error &e)
  {
    std::cerr << fmt::format("Failed to read OBJ file '{0}'\n\t{1}", input_file, e.what()) << std::endl;
    return EXIT_FAILURE;
  }

  std::cerr << "Triangle count: " << triangles.size() << std::endl;

  const auto start_bvh = steady_clock::now();

  BVH bvh(triangles);

  if (bvh.failed())
  {
    std::cerr << "BVH construction failed" << std::endl;
    return EXIT_FAILURE;
  }

  const auto end_bvh = steady_clock::now();

  std::cerr << fmt::format("BVH construction took {0} ms\n", duration_cast<milliseconds>(end_bvh - start_bvh).count());

  Frame frame(1024, 768);

  const auto start = steady_clock::now();

  int n_hit = 0;

  for (int i = 0; i < FRAME_HEIGHT; i++)
  {
    const float y = -(VIEWPORT_HEIGHT / 2.0f) + (i * DY);
    float x = -(VIEWPORT_WIDTH / 2.0f);

    RGBA *p = frame.pixels.get() + (FRAME_HEIGHT - i - 1)*FRAME_WIDTH;

    for (int j = 0; j < FRAME_WIDTH; j++)
    {
      Ray ray = { { x, y, 1.0f }, { 0.0f, 0.0f, -1.0f } };
      const bool hit = bvh.intersect(ray);

      const uint8_t c = (hit ? static_cast<uint8_t>(std::abs(ray.dot) * 255.0f) : 0);

      *p = RGBA{ c, c, c, 255 };

      n_hit += hit;
      x += DX;
      p += 1;
    }
  }

  const auto end = steady_clock::now();

  std::cerr << "N HIT: " << n_hit << std::endl;

  std::cerr << std::setprecision(12)
            << N_RAYS << " intersections took: " << duration_cast<milliseconds>(end - start).count() << "ms ("
            << N_RAYS * (1'000'000.0 / duration_cast<microseconds>(end - start).count()) << " rays/second)" << std::endl;

  utils::Ppm::write("out.ppm", frame);

  return EXIT_SUCCESS;
}
