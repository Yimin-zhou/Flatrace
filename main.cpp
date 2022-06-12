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

constexpr auto FRAME_WIDTH = 1024;
constexpr auto FRAME_HEIGHT = 768;

constexpr float VIEWPORT_WIDTH = 1.2f;
constexpr float VIEWPORT_HEIGHT = 1.2f;

constexpr float DX = VIEWPORT_WIDTH / FRAME_WIDTH;
constexpr float DY = VIEWPORT_HEIGHT / FRAME_HEIGHT;

constexpr int TILE_SIZE = 16;

constexpr auto N_FRAMES = 1;
constexpr auto N_RAYS = N_FRAMES * FRAME_WIDTH * FRAME_HEIGHT;

namespace {

void render_frame(const BVH &bvh, RGBA * const frameBuffer)
{
  for (int tile_i = 0; tile_i < (FRAME_HEIGHT / TILE_SIZE); tile_i++)
  {
    const float tile_y = -(VIEWPORT_HEIGHT / 2.0f) + (tile_i * TILE_SIZE * DY);

    for (int tile_j = 0; tile_j < (FRAME_WIDTH / TILE_SIZE); tile_j++)
    {
      const float tile_x = -(VIEWPORT_WIDTH / 2.0f) + (tile_j * TILE_SIZE * DX);

      float y = tile_y;
      RGBA * p = frameBuffer + (FRAME_HEIGHT - tile_i*TILE_SIZE - 1)*FRAME_WIDTH + (tile_j*TILE_SIZE);

      for (int i = tile_i*TILE_SIZE; i < (tile_i*TILE_SIZE) + TILE_SIZE; i++)
      {
        float x = tile_x;

        for (int j = tile_j*TILE_SIZE; j < (tile_j*TILE_SIZE) + TILE_SIZE; j++)
        {
          Ray ray = { { x, y, 1.0f }, { 0.0f, 0.0f, -1.0f } };
          const bool hit = bvh.intersect(ray);

          const uint8_t c = (hit ? static_cast<uint8_t>(std::abs(ray.dot) * 255.0f) : 0);

          *p = RGBA{ c, c, c, 255 };

          x += DX;
          p += 1;
        }

        y += DY;
        p -= (FRAME_WIDTH + TILE_SIZE);
      }
    }
  }
}


void render_frame_2x2(const BVH &bvh, RGBA * const frameBuffer)
{
  for (int tile_i = 0; tile_i < FRAME_HEIGHT; tile_i += TILE_SIZE)
  {
    for (int tile_j = 0; tile_j < FRAME_WIDTH; tile_j += TILE_SIZE)
    {
      for (int bundle_i = tile_i; bundle_i < (tile_i + TILE_SIZE); bundle_i += 2)
      {
        const float bundle_y = -(VIEWPORT_HEIGHT / 2.0f) + (bundle_i * DY);

        for (int bundle_j = tile_j; bundle_j < (tile_j + TILE_SIZE); bundle_j += 2)
        {
          const float bundle_x = -(VIEWPORT_WIDTH / 2.0f) + (bundle_j * DX);

          RGBA * const p = frameBuffer + (FRAME_HEIGHT - bundle_i- 1)*FRAME_WIDTH + bundle_j;

          Ray2x2 rays({ bundle_x, bundle_y, 1.0f }, { 0.0f, 0.0f, -1.0f }, DX, DY);

          const uint8_t hit = bvh.intersect2x2(rays);

          const uint8_t c0 = ((hit & 0b0001) ? static_cast<uint8_t>(std::abs(rays.dot[0]) * 255.0f) : 0);
          const uint8_t c1 = ((hit & 0b0010) ? static_cast<uint8_t>(std::abs(rays.dot[1]) * 255.0f) : 0);
          const uint8_t c2 = ((hit & 0b0100) ? static_cast<uint8_t>(std::abs(rays.dot[2]) * 255.0f) : 0);
          const uint8_t c3 = ((hit & 0b1000) ? static_cast<uint8_t>(std::abs(rays.dot[3]) * 255.0f) : 0);

          *p =  RGBA{ c0, 0, 0, 255 };
          *(p + 1) =  RGBA{ c1, 0, 0, 255 };
          *(p - FRAME_WIDTH) =  RGBA{ c2, 0, 0, 255 };
          *(p - FRAME_WIDTH + 1) =  RGBA{ c3, 0, 0, 255 };
        }
      }
    }
  }
}

}

int main(int argc, char **argv)
{
  using namespace std::chrono;

  std::cerr << sizeof(core::Vec3) << std::endl;
  std::cerr << sizeof(core::Ray2x2) << std::endl;

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

  for (int i = 0; i < 10; i++)
  {
    const auto start = steady_clock::now();

#if 0
    render_frame(bvh, frame.pixels.get());
#else
    render_frame_2x2(bvh, frame.pixels.get());
#endif

    const auto end = steady_clock::now();

    std::cerr << std::setprecision(12)
              << N_RAYS << " intersections took: " << duration_cast<milliseconds>(end - start).count() << "ms ("
              << N_RAYS * (1'000'000.0 / duration_cast<microseconds>(end - start).count()) << " rays/second)" << std::endl;
  }

  utils::Ppm::write("out.ppm", frame);

  return EXIT_SUCCESS;
}
