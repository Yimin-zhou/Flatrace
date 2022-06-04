#include "core/types.h"
#include "core/intersect.h"
#include "core/frame.h"

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

  constexpr float DX = 1.0f / FRAME_WIDTH;
  constexpr float DY = 1.0f / FRAME_HEIGHT;

  constexpr auto N_FRAMES = 1;
  constexpr auto N_RAYS = N_FRAMES * FRAME_WIDTH * FRAME_HEIGHT;

  using namespace std::chrono;

  if (argc != 2)
  {
    std::cerr << "\nUsage: flatrace <mesh_file.obj>\n\n";
    return EXIT_FAILURE;
  }

  const std::string input_file(argv[1]);

  // Load triangle data
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

  const Triangle triangle({ 0.0f, -0.1f, 1.0f }, { -0.1f, 0.1f, 1.0f }, { 0.1f, 0.1f, 1.0f });

  Frame frame(1024, 768);

  const auto start = steady_clock::now();

  int n_hit = 0;

  for (int i = 0; i < FRAME_HEIGHT; i++)
  {
    const float y = -0.5f + (i * DY);
    float x = -0.5f;

    RGBA *p = frame.pixels.get() + i*FRAME_WIDTH;

    for (int j = 0; j < FRAME_WIDTH; j++)
    {
      const bool hit = intersect(triangle, { { x, y, 0.0f }, { 0.0f, 0.0f, 1.0f } });

      *p = (hit ? RGBA{ 255, 255, 255, 255 } : RGBA{ 0, 0, 0, 255 });

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
