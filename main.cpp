#include "src/core/trace.h"
#include "src/debug/bvh_debugger.h"

int main(int argc, char **argv)
{
    using namespace std::chrono;
    using namespace core;

    // if (argc < 2)
    // {
    //   std::cerr << "\nUsage: ./flatrace <mesh_file.obj>\n\n";
    //   return EXIT_FAILURE;
    // }
    // const std::string input_file(argv[1]);
    const bool flip = (argc == 3) && (argv[2][0] == '1');

    // Set a default model
    const std::string input_file("/home/fries/thesis/code/Flatrace/test/input/bunny.obj");

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
            const glm::vec3 &v0f = { t.vertices[0].x, t.vertices[0].z, t.vertices[0].y };
            const glm::vec3 &v1f = { t.vertices[1].x, t.vertices[1].z, t.vertices[1].y };
            const glm::vec3 &v2f = { t.vertices[2].x, t.vertices[2].z, t.vertices[2].y };

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
    SDL_WINDOW_ALLOW_HIGHDPI | SDL_WINDOW_RESIZABLE);

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
    float theta = 2.0f;

    float max_rps = -INF;
    float min_rps = INF;

    // For debugging BVH nodes
    int maxDepth = bvh.getMaxDepth();
    int nodeCount = bvh.getNodes().size();

    while (!quit)
    {
        // Handle pending events
        SDL_Event e;

        while (SDL_PollEvent(&e))
        {
          ImGui_ImplSDL2_ProcessEvent(&e);

          switch (e.type)
          {
              case SDL_WINDOWEVENT:
              {
                  if (e.window.event == SDL_WINDOWEVENT_RESIZED)
                  {
                      // maintain aspect ratio
                      SDL_RenderSetLogicalSize(renderer, WINDOW_WIDTH, WINDOW_HEIGHT);

                      SDL_RenderSetScale(renderer, (float) e.window.data1 / WINDOW_WIDTH, (float) e.window.data2 / WINDOW_HEIGHT);
                  }
                  break;
              }
              case SDL_QUIT:
                  quit = true;
                  break;
          }
        }

        // Update camera
        const float cx = std::cos(theta) * 2.0f;
        const float cz = std::sin(theta) * 2.0f;

        Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};

        theta += (2.0f * M_PI / 120.0f) * SPEED;

        // Render frame
        const auto start = steady_clock::now();

        #if 1
        render_frame(camera, bvh, frame.pixels.get(), maxDepth);
        #else
        render_frame_4x4(camera, bvh, frame.pixels.get());
        #endif

        // TODO draw bvh visualization
        // debug::draw_bvh(renderer, bvh);

        const auto end = steady_clock::now();

        // Compute stats
        const int us = duration_cast<microseconds>(end - start).count();
        const int ms = us / 1000;
        const float rps = ((float) N_RAYS / us); // current frame rps
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

#ifdef DEBUG
        {
            ImGui::Begin("BVH Viewer");
            ImGui::Text("Number of Nodes: %d", nodeCount);
            ImGui::Text("Max Depth: %d", maxDepth);
            ImGui::Separator();
            if (ImGui::CollapsingHeader("BVH Nodes", ImGuiTreeNodeFlags_DefaultOpen))
            {
                debug::renderBVHtree(bvh.getRoot(), bvh.getNodes());
            }
            ImGui::End();
        }
#endif

        {
            ImGui::Begin("Render properties", nullptr);

            if (!ImGui::IsWindowCollapsed()) {
                ImGui::Text("Frame rate:");
                ImGui::Text("%s",
                            fmt::format("{0} ms, {1} fps", ms, fps).c_str());
                ImGui::Separator();

                ImGui::Text("Rays:");
                ImGui::Text("%s", fmt::format("{1:.2f}M rps, min/max rps: {0:.2f}M/{1:.2f}M", ((double) N_RAYS / us), min_rps, max_rps).c_str());
                ImGui::Separator();

                ImGui::Text("Memory usage:");
                size_t memoryUsage = debug::getCurrentRSS(); // in bytes
                ImGui::Text("%s", fmt::format("{0:.5f} MB", ((double) memoryUsage / 1024 / 1024)).c_str());
                ImGui::Separator();

            }

            ImGui::End();
        }

        ImGui::Render();

        ImGui_ImplSDLRenderer_RenderDrawData(ImGui::GetDrawData());

        SDL_RenderPresent(renderer);
    }

    return EXIT_SUCCESS;
}
