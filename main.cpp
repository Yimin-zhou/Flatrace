#include "src/core/types.h"
#include "src/core/frame.h"
#include "src/core/bvh.h"

#include "src/utils/ppm.h"
#include "src/utils/obj.h"

#include "src/debug/visualization.h"
#include "src/debug/bvhCounter.h"

#include <SDL2/SDL.h>

#include <tbb/parallel_for.h>

#include <imgui/imgui.h>
#include <imgui/imgui_impl_sdl.h>
#include <imgui/imgui_impl_sdlrenderer.h>

#include <fmt/format.h>

#include <chrono>
#include <iostream>

using namespace core;

// TODO this should for each ray
std::atomic<int> BVHNodeCounter::counter(0);

namespace {
constexpr auto WINDOW_WIDTH = 1024;
constexpr auto WINDOW_HEIGHT = 768;

constexpr auto FRAME_WIDTH = 1024;
constexpr auto FRAME_HEIGHT = 768;

constexpr auto VIEWPORT_WIDTH  = 1.2f;
constexpr auto VIEWPORT_HEIGHT = 1.2f;

constexpr auto DX = VIEWPORT_WIDTH / FRAME_WIDTH;
constexpr auto DY = VIEWPORT_HEIGHT / FRAME_HEIGHT;

constexpr auto TILE_SIZE = 16;
constexpr auto BUNDLE_SIZE = 4;

constexpr auto NX = FRAME_WIDTH / TILE_SIZE;
constexpr auto NY = FRAME_HEIGHT / TILE_SIZE;

constexpr auto N_FRAMES = 1;
constexpr auto N_RAYS = N_FRAMES * FRAME_WIDTH * FRAME_HEIGHT;

constexpr auto MAX_INTERSECTIONS = 3;

constexpr auto SPEED = 0.05f;

// Arbitrary color palette for materials
static const std::array<std::array<float, 4>, 8> COLORS = { {
  { { 1.0f, 0.0f, 0.0f, 1.0f } },
  { { 0.0f, 1.0f, 0.0f, 1.0f } },
  { { 0.0f, 0.0f, 1.0f, 1.0f } },
  { { 1.0f, 1.0f, 0.0f, 1.0f } },
  { { 1.0f, 0.0f, 1.0f, 1.0f } },
  { { 0.0f, 1.0f, 1.0f, 1.0f } },
  { { 1.0f, 0.5f, 0.5f, 1.0f } },
  { { 0.5f, 0.5f, 1.0f, 1.0f } },
} };

// Reference implementation that traces 1 ray at a time (no SIMD)
void render_frame(const Camera &camera, const BVH &bvh, RGBA * const frameBuffer)
{
  tbb::parallel_for(tbb::blocked_range<int>(0, NX*NY), [&](const tbb::blocked_range<int> &r)
  {
    for (int tile_idx = r.begin(); tile_idx != r.end(); tile_idx++)
    {
      const int tile_i = (tile_idx / NX);
      const int tile_j = (tile_idx % NX);

      const float tile_y = -(VIEWPORT_HEIGHT / 2.0f) + (tile_i * TILE_SIZE * DY);
      const float tile_x = -(VIEWPORT_WIDTH / 2.0f) + (tile_j * TILE_SIZE * DX);

      float y = tile_y;
      RGBA * p = frameBuffer + (FRAME_HEIGHT - tile_i*TILE_SIZE - 1)*FRAME_WIDTH + (tile_j*TILE_SIZE);
      // TODO change color
      for (int i = tile_i*TILE_SIZE; i < (tile_i*TILE_SIZE) + TILE_SIZE; i++)
      {
        float x = tile_x;

        for (int j = tile_j*TILE_SIZE; j < (tile_j*TILE_SIZE) + TILE_SIZE; j++)
        {
          const Vec3 ray_origin = camera.p + camera.x*x + camera.y*y;
          const Vec3 ray_direction = camera.d;

          Ray ray = { ray_origin, ray_direction };

          const bool hit = bvh.intersect(ray, MAX_INTERSECTIONS);

          const float src_alpha = 0.6f;

          __m128i c = _mm_set1_epi32(0);

          if (hit)
          {
            __m128 cf = _mm_set1_ps(0.0f);

            float dst_alpha = 1.0f;

            for (int n = 0; n < 3; n++)
            {
              const int triangle = ray.triangle[n];

              // c += COLORS[bvh.getTriangle(ray.triangle[0]).material & 0x07] * (dst_alpha * src_alpha * std::abs(ray.dot[n]));
              const __m128 abs_dot_x4 = _mm_andnot_ps(_mm_set1_ps(-0.0f), _mm_load1_ps(&ray.dot[n]));
              const __m128 tri_color = _mm_load_ps(COLORS[bvh.getTriangle(triangle).material & 0x07].data());
              const __m128 shaded_color = _mm_mul_ps(tri_color, abs_dot_x4);

              const float alpha = dst_alpha * src_alpha;

              cf = _mm_add_ps(cf, _mm_mul_ps(_mm_load1_ps(&alpha), shaded_color));

              dst_alpha *= (1.0 - src_alpha);
            }

            // c = min(255 * cf)
            cf = _mm_min_ps(_mm_mul_ps(cf, _mm_set1_ps(255.0f)), _mm_set1_ps(255.0f));
            c = _mm_shuffle_epi8(_mm_cvtps_epi32(cf), _mm_set1_epi32(0x0C080400));
          }

          // *p = c;
          _mm_storeu_si32(p, c);

          x += DX;
          p += 1;
        }

        y += DY;
        p -= (FRAME_WIDTH + TILE_SIZE);
      }
    }
  });
}

// 8-way SIMD implementation that traces 4x4 'ray bundles'
void render_frame_4x4(const Camera &camera, const BVH &bvh, RGBA * const frameBuffer)
{
  const Vec3 rd = { 1.0f / camera.d.x, 1.0f / camera.d.y, 1.0f / camera.d.z };

  tbb::parallel_for(tbb::blocked_range<int>(0, NX*NY), [&](const tbb::blocked_range<int> &r)
  {
    for (int tile_idx = r.begin(); tile_idx != r.end(); tile_idx++)
    {
      const int tile_i = (tile_idx / NX);
      const int tile_j = (tile_idx % NX);

      for (int bundle_i = 0; bundle_i < TILE_SIZE/BUNDLE_SIZE; bundle_i++)
      {
        const int bundle_py = (tile_i * TILE_SIZE) + (bundle_i * BUNDLE_SIZE);
        const float bundle_y = -(VIEWPORT_HEIGHT / 2.0f) + (bundle_py * DY);

        for (int bundle_j = 0; bundle_j < TILE_SIZE/BUNDLE_SIZE; bundle_j++)
        {
          const int bundle_px = (tile_j * TILE_SIZE) + (bundle_j * BUNDLE_SIZE);
          const float bundle_x = -(VIEWPORT_WIDTH / 2.0f) + (bundle_px * DX);

          RGBA * const p = frameBuffer + ((FRAME_HEIGHT - bundle_py - BUNDLE_SIZE) * FRAME_WIDTH) + bundle_px;

          const Vec3 bundle_origin = camera.p + camera.x*bundle_x + camera.y*bundle_y;

          Ray4x4 rays = { camera, bundle_origin, camera.d, rd, DX, DY };

          const bool hit = bvh.intersect4x4(rays, MAX_INTERSECTIONS);

          __m128 src_alpha = _mm_set1_ps(1.0f);

          for (int r = 0; r < 16; r++)
          {
            const int ray_i = r / 4;
            const int ray_j = r % 4;

            __m128i c = _mm_set1_epi32(0);

            if (hit)
            {
              __m128 dst_alpha = _mm_set1_ps(1.0f);

              __m128 cf = _mm_set1_ps(0.0f);

              for (int n = 0; n < 3; n++)
              {
                const int triangle = rays.triangle[n*16 + r];

                // c += COLORS[bvh.getTriangle(ray.triangle[0]).material & 0x07] * (dst_alpha * src_alpha * std::abs(ray.dot[n]));
                const __m128 abs_dot_x4 = _mm_andnot_ps(_mm_set1_ps(-0.0f), _mm_load1_ps(rays.dot.data() + n*16 + r));
                const __m128 tri_color = _mm_load_ps(COLORS[bvh.getTriangle(triangle).material & 0x07].data());
                const __m128 shaded_color = _mm_mul_ps(tri_color, abs_dot_x4);

                const __m128 alpha = _mm_mul_ps(dst_alpha, src_alpha);

                cf = _mm_add_ps(cf, _mm_mul_ps(alpha, shaded_color));

                // dst_alpha *= (1.0 - src_alpha);
                dst_alpha = _mm_mul_ps(dst_alpha, _mm_sub_ps(_mm_set1_ps(1.0f), src_alpha));
              }

//              // For debugging: Retrieve the number of nodes this ray has traversed.
//              {
//                    int nodesTraversed = rays.bvh_nodes_visited[r];  // assuming nodes_visited is an array of 16 ints
//                    // Modify color based on the number of traversed nodes.
//                    float factor = static_cast<float>(nodesTraversed) / 1.0f;  // Convert to a suitable factor, e.g., [0.0, 1.0]
//                    factor = (factor > 1.0f) ? 1.0f : factor;  // clamp the value
//                    __m128 factor_vec = _mm_set1_ps(factor);
//                    __m128 red_channel = _mm_shuffle_ps(cf, cf, _MM_SHUFFLE(0, 0, 0, 0)); // isolate red channel
//                    // Apply the factor only to the red channel
//                    red_channel = _mm_mul_ps(red_channel, factor_vec);
//
//                    // Combine back into the original color
//                    __m128 green_blue_alpha = _mm_shuffle_ps(cf, cf,
//                                                             _MM_SHUFFLE(3, 2, 1, 1)); // keep green, blue, and alpha
//                    cf = _mm_move_ss(green_blue_alpha, red_channel); // replace red with modified value
//              }
                // c = min(255 * cf)
              cf = _mm_min_ps(_mm_mul_ps(cf, _mm_set1_ps(255.0f)), _mm_set1_ps(255.0f));
              c = _mm_shuffle_epi8(_mm_cvtps_epi32(cf), _mm_set1_epi32(0x0C080400));
            }

            _mm_storeu_si32(p + ((BUNDLE_SIZE - ray_i - 1) * FRAME_WIDTH) + ray_j, c);
          }
        }
      }
    }
  });
}

}

int main(int argc, char **argv)
{
  using namespace std::chrono;

  // if (argc < 2)
  // {
  //   std::cerr << "\nUsage: ./flatrace <mesh_file.obj>\n\n";
  //   return EXIT_FAILURE;
  // }
  // const std::string input_file(argv[1]);
  const bool flip = (argc == 3) && (argv[2][0] == '1');
  
  // Set a default model
  const std::string input_file("/home/fries/thesis/code/Flatrace/test/input/Luxury_House.obj");

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

  // Flip y/z coordinates if '1' was passed on the command line, after the input file name
  if (flip)
  {
    std::transform(triangles.begin(), triangles.end(), triangles.begin(), [](const Triangle &t) -> Triangle
    {
      const Vec3 &v0f = { t.vertices[0].x, t.vertices[0].z, t.vertices[0].y };
      const Vec3 &v1f = { t.vertices[1].x, t.vertices[1].z, t.vertices[1].y };
      const Vec3 &v2f = { t.vertices[2].x, t.vertices[2].z, t.vertices[2].y };

      return { t.id, v0f, v2f, v1f, t.material };
    });
  }

  const auto start_bvh = steady_clock::now();

  BVH bvh(triangles);

  if (bvh.failed())
  {
    std::cerr << "BVH construction failed" << std::endl;
    return EXIT_FAILURE;
  }

  const auto end_bvh = steady_clock::now();

  std::cerr << fmt::format("BVH construction took {0} ms\n", duration_cast<milliseconds>(end_bvh - start_bvh).count());

    // Initialize SDL
  if (SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    std::cout << "Failed to initialize SDL2" << std::endl;
    return EXIT_FAILURE;
  }

  // Create SDL window and renderer
  SDL_Window * const window = SDL_CreateWindow(
    "flatrace",
    SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
    WINDOW_WIDTH, WINDOW_HEIGHT,
    SDL_WINDOW_ALLOW_HIGHDPI);

  SDL_Renderer * const renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC);

  SDL_RenderSetVSync(renderer, 1);

  // Check window size vs drawable size. On macOS HiDPI screens, these will be different, and
  // everything will be rendered ridiculously small unless we enable render scaling.
  {
    int window_width, window_height;
    int drawable_width, drawable_height;

    SDL_GetWindowSize(window, &window_width, &window_height);
    SDL_GetRendererOutputSize(renderer, &drawable_width, &drawable_height);

    SDL_RenderSetScale(renderer, (float) drawable_width / window_width, (float) drawable_height / window_height);
  }

  // The framebuffer will be streamed to a texture which is blitted to the SDL window at each frame
  SDL_Texture *framebuffer_texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ABGR8888, SDL_TEXTUREACCESS_STREAMING, FRAME_WIDTH, FRAME_HEIGHT);

  // Init ImGui
  ImGui::CreateContext();

  ImGuiIO &io = ImGui::GetIO();
  (void) io;

  io.IniFilename = nullptr;

  ImGui_ImplSDL2_InitForSDLRenderer(window);
  ImGui_ImplSDLRenderer_Init(renderer);

  // Framebuffer for the raytracer
  Frame frame(FRAME_WIDTH, FRAME_HEIGHT);

  // Main render loop
  bool quit = false;
  float theta = 0.0f;

  float max_rps = -INF;
  float min_rps = INF;

  while (!quit)
  {
    // Handle pending events
    SDL_Event e;

    while (SDL_PollEvent(&e))
    {
      ImGui_ImplSDL2_ProcessEvent(&e);

      switch (e.type)
      {
        case SDL_QUIT:
          quit = true;
          break;
      }
    }

    const float cx = std::cos(theta) * 2.0f;
    const float cz = std::sin(theta) * 2.0f;

    Camera camera = { { cx, 1.0f, cz }, { -cx, -1.0f, -cz }, { 0.0f, 1.0f, 0.0f }, 5.0f };

    theta += (2.0f*M_PI / 120.0f) * SPEED;

    const auto start = steady_clock::now();



#if 0
    render_frame(camera, bvh, frame.pixels.get());
#else
    render_frame_4x4(camera, bvh, frame.pixels.get());
#endif

    // TODO draw bvh visualization
    // debug::draw_bvh(renderer, bvh);

    const auto end = steady_clock::now();

    const int us = duration_cast<microseconds>(end - start).count();
    const int ms = us / 1000;
    const float rps = ((float) N_RAYS / us);
    const float fps = 1000 / ms;

    max_rps = std::max(max_rps, rps);
    min_rps = std::min(min_rps, rps);

    // Update output texture and blit to window
    SDL_UpdateTexture(framebuffer_texture, nullptr, (void *) frame.pixels.get(), FRAME_WIDTH*4);

    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, framebuffer_texture, nullptr, nullptr);

    // Add ImGui overlays
    ImGui_ImplSDLRenderer_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin("Render time", nullptr);

    if (!ImGui::IsWindowCollapsed())
    {
      ImGui::Text("%s", fmt::format("{0} ms, {1} fps, {1:.2f}M rps", ms, fps, ((double) N_RAYS / us)).c_str());
      ImGui::Text("%s", fmt::format("min/max rps: {0:.2f}M/{1:.2f}M", min_rps, max_rps).c_str());
      ImGui::Text("%s", fmt::format("BVH node count: {0}", BVHNodeCounter::getCount()).c_str());
    }

    ImGui::End();

    ImGui::Render();

    ImGui_ImplSDLRenderer_RenderDrawData(ImGui::GetDrawData());

    SDL_RenderPresent(renderer);
  }

  return EXIT_SUCCESS;
}
