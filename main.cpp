#include "core/types.h"
#include "core/frame.h"
#include "core/bvh.h"

#include "utils/ppm.h"
#include "utils/obj.h"

#include <SDL2/SDL.h>

#include <imgui/imgui.h>
#include <imgui/imgui_impl_sdl.h>
#include <imgui/imgui_impl_sdlrenderer.h>
#include <fmt/format.h>

#include <chrono>
#include <iostream>

using namespace core;

constexpr auto WINDOW_WIDTH = 1024;
constexpr auto WINDOW_HEIGHT = 768;

constexpr auto FRAME_WIDTH = 1024;
constexpr auto FRAME_HEIGHT = 768;

constexpr float VIEWPORT_WIDTH  = 1.4f;
constexpr float VIEWPORT_HEIGHT = 1.4f;

constexpr float DX = VIEWPORT_WIDTH / FRAME_WIDTH;
constexpr float DY = VIEWPORT_HEIGHT / FRAME_HEIGHT;

constexpr int TILE_SIZE = 16;

constexpr auto N_FRAMES = 1;
constexpr auto N_RAYS = N_FRAMES * FRAME_WIDTH * FRAME_HEIGHT;

namespace {

void render_frame(const Camera &camera, const BVH &bvh, RGBA * const frameBuffer)
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
          const Vec3 ray_origin = camera.p + camera.x*x + camera.y*y;
          const Vec3 ray_direction = camera.d; //(ray_origin - (camera.p - camera.d) * camera.zoom).normalized();

          Ray ray = { ray_origin, ray_direction };

          const bool hit = bvh.intersect(ray);

          const uint8_t c = (hit ? static_cast<uint8_t>(std::abs(ray.dot) * 200.0f) : 0);

          *p = RGBA{ c, uint8_t(c >> 1), c, 255 };

          x += DX;
          p += 1;
        }

        y += DY;
        p -= (FRAME_WIDTH + TILE_SIZE);
      }
    }
  }
}

void render_frame_4x4(const Camera &camera, const BVH &bvh, RGBA * const frameBuffer)
{
  for (int tile_i = 0; tile_i < FRAME_HEIGHT; tile_i += TILE_SIZE)
  {
    for (int tile_j = 0; tile_j < FRAME_WIDTH; tile_j += TILE_SIZE)
    {
      for (int bundle_i = tile_i; bundle_i < (tile_i + TILE_SIZE); bundle_i += 4)
      {
        const float bundle_y = -(VIEWPORT_HEIGHT / 2.0f) + (bundle_i * DY);

        for (int bundle_j = tile_j; bundle_j < (tile_j + TILE_SIZE); bundle_j += 4)
        {
          const float bundle_x = -(VIEWPORT_WIDTH / 2.0f) + (bundle_j * DX);

          RGBA * const p = frameBuffer + (FRAME_HEIGHT - bundle_i- 1)*FRAME_WIDTH + bundle_j;

          const Vec3 bundle_origin = camera.p + camera.x*bundle_x + camera.y*bundle_y;
          const Vec3 ray_direction = camera.d; //(ray_origin - (camera.p - camera.d) * camera.zoom).normalized();

          Ray4x4 rays = { camera, bundle_origin, ray_direction, DX, DY };

          const int hit = bvh.intersect4x4(rays);

          // TODO: obviously the unrolled 4x4 pixel handling should also be vectorized....
          const uint8_t c0  = ((hit & (1 << 0)) ? static_cast<uint8_t>(std::abs(rays.dot[0]) * 255.0f) : 0);
          const uint8_t c1  = ((hit & (1 << 1)) ? static_cast<uint8_t>(std::abs(rays.dot[1]) * 255.0f) : 0);
          const uint8_t c2  = ((hit & (1 << 2)) ? static_cast<uint8_t>(std::abs(rays.dot[2]) * 255.0f) : 0);
          const uint8_t c3  = ((hit & (1 << 3)) ? static_cast<uint8_t>(std::abs(rays.dot[3]) * 255.0f) : 0);

          const uint8_t c4  = ((hit & (1 << 4)) ? static_cast<uint8_t>(std::abs(rays.dot[4]) * 255.0f) : 0);
          const uint8_t c5  = ((hit & (1 << 5)) ? static_cast<uint8_t>(std::abs(rays.dot[5]) * 255.0f) : 0);
          const uint8_t c6  = ((hit & (1 << 6)) ? static_cast<uint8_t>(std::abs(rays.dot[6]) * 255.0f) : 0);
          const uint8_t c7  = ((hit & (1 << 7)) ? static_cast<uint8_t>(std::abs(rays.dot[7]) * 255.0f) : 0);

          const uint8_t c8  = ((hit & (1 << 8)) ? static_cast<uint8_t>(std::abs(rays.dot[8]) * 255.0f) : 0);
          const uint8_t c9  = ((hit & (1 << 9)) ? static_cast<uint8_t>(std::abs(rays.dot[9]) * 255.0f) : 0);
          const uint8_t c10 = ((hit & (1 << 10)) ? static_cast<uint8_t>(std::abs(rays.dot[10]) * 255.0f) : 0);
          const uint8_t c11 = ((hit & (1 << 11)) ? static_cast<uint8_t>(std::abs(rays.dot[11]) * 255.0f) : 0);

          const uint8_t c12 = ((hit & (1 << 12)) ? static_cast<uint8_t>(std::abs(rays.dot[12]) * 255.0f) : 0);
          const uint8_t c13 = ((hit & (1 << 13)) ? static_cast<uint8_t>(std::abs(rays.dot[13]) * 255.0f) : 0);
          const uint8_t c14 = ((hit & (1 << 14)) ? static_cast<uint8_t>(std::abs(rays.dot[14]) * 255.0f) : 0);
          const uint8_t c15 = ((hit & (1 << 15)) ? static_cast<uint8_t>(std::abs(rays.dot[15]) * 255.0f) : 0);

          *p =  RGBA{ c0, 0, 0, 255 };
          *(p + 1) =  RGBA{ c1, 0, 0, 255 };
          *(p + 2) =  RGBA{ c2, 0, 0, 255 };
          *(p + 3) =  RGBA{ c3, 0, 0, 255 };

          *(p - FRAME_WIDTH) =  RGBA{ c4, 0, 0, 255 };
          *(p - FRAME_WIDTH + 1) =  RGBA{ c5, 0, 0, 255 };
          *(p - FRAME_WIDTH + 2) =  RGBA{ c6, 0, 0, 255 };
          *(p - FRAME_WIDTH + 3) =  RGBA{ c7, 0, 0, 255 };

          *(p - 2*FRAME_WIDTH) =  RGBA{ c8, 0, 0, 255 };
          *(p - 2*FRAME_WIDTH + 1) =  RGBA{ c9, 0, 0, 255 };
          *(p - 2*FRAME_WIDTH + 2) =  RGBA{ c10, 0, 0, 255 };
          *(p - 2*FRAME_WIDTH + 3) =  RGBA{ c11, 0, 0, 255 };

          *(p - 3*FRAME_WIDTH) =  RGBA{ c12, 0, 0, 255 };
          *(p - 3*FRAME_WIDTH + 1) =  RGBA{ c13, 0, 0, 255 };
          *(p - 3*FRAME_WIDTH + 2) =  RGBA{ c14, 0, 0, 255 };
          *(p - 3*FRAME_WIDTH + 3) =  RGBA{ c15, 0, 0, 255 };
        }
      }
    }
  }
}

}

int main(int argc, char **argv)
{
  using namespace std::chrono;

  if (argc < 2)
  {
    std::cerr << "\nUsage: flatrace <mesh_file.obj>\n\n";
    return EXIT_FAILURE;
  }

  const std::string input_file(argv[1]);

  const bool flip = (argc == 3) && (argv[2][0] == '1');

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

      return Triangle(v0f, v2f, v1f);
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
  SDL_Texture *framebuffer_texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, FRAME_WIDTH, FRAME_HEIGHT);

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

    theta += (2.0f*M_PI / 120.0f);

    const auto start = steady_clock::now();

#if 0
    render_frame(camera, bvh, frame.pixels.get());
#else
    render_frame_4x4(camera, bvh, frame.pixels.get());
#endif

    const auto end = steady_clock::now();

    const auto us = duration_cast<microseconds>(end - start).count();

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
      ImGui::Text("%s", fmt::format("{0} ms, {1:.2f}M rays/second", us / 1000, ((double) N_RAYS / us)).c_str());
    }

    ImGui::End();

    ImGui::Render();

    ImGui_ImplSDLRenderer_RenderDrawData(ImGui::GetDrawData());

    SDL_RenderPresent(renderer);
  }

  return EXIT_SUCCESS;
}
