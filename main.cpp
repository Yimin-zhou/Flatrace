#include "core/types.h"
#include "core/intersect.h"

#include <vector>
#include <chrono>
#include <iostream>
#include <iomanip>

using namespace core;

int main(int argc, char **argv)
{
  constexpr auto N_RAYS_BITS = 8;
  constexpr auto N_RAYS = (1 << N_RAYS_BITS);

  using namespace std::chrono;

  const Triangle triangle({ 0.0f, -0.5f, 1.0f }, { 0.5f, 0.5f, 1.0f }, { -0.5f, 0.5f, 1.0f });

  std::vector<Ray> rays(N_RAYS);

  for (auto i = 0; i < N_RAYS; i++)
  {
    const float x = ((float) rand() / RAND_MAX) - 0.5f;
    const float y = ((float) rand() / RAND_MAX) - 0.5f;

    rays[i] = { { x, y, 0.0f }, { 0.0f, 0.0f, -1.0f } };
  }

  const auto start = steady_clock::now();

  int n_hit = 0;

  for (size_t i = 0; i < 1'000'000'000; i++)
  {
    n_hit += intersect(triangle, rays[i & (N_RAYS_BITS - 1)]);
  }

  const auto end = steady_clock::now();

  std::cerr << "N HIT: " << n_hit << std::endl;

  std::cerr << std::setprecision(12)
            << "1G intersections took: " << duration_cast<milliseconds>(end - start).count() << "ms ("
            << 1'000'000'000 * (1'000'000.0 / duration_cast<microseconds>(end - start).count()) << " rays/second)" << std::endl;


  return EXIT_SUCCESS;
}
