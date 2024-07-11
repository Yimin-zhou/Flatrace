#include "src/core/trace.h"
#include "src/debug/bvh_debugger.h"
#include "src/utils/globalState.h"
#include "src/utils/obj.h"
#include <Tracy.hpp>

int main()
{
    using namespace std::chrono;
    using namespace core;

    // Set a default model
    const std::string input_folder("test/input/small_semi");

    // Load getTriangle data
    std::vector<std::vector<Triangle>> models;

    try
    {
        models = utils::Obj::loadAllObjFilesInFolder(input_folder, MODEL_NORMALIZE);
    }
    catch (std::runtime_error &e)
    {
        std::cerr << fmt::format("Failed to read OBJ file '{0}'\n\t{1}", input_folder, e.what()) << std::endl;
        return EXIT_FAILURE;
    }

    std::vector<Triangle> triangles;
    for (const auto &model : models)
    {
        triangles.insert(triangles.end(), model.begin(), model.end());
    }

    std::cerr << "Triangle count: " << triangles.size() << std::endl;

    // Flip y/z coordinates if '1' was passed on the command line, after the input file name
    if (MODEL_FLIP)
    {
        std::transform(triangles.begin(), triangles.end(), triangles.begin(), [](const Triangle &t) -> Triangle
        {
            const glm::vec3 &v0f = { t.vertices[0].x, t.vertices[0].z, t.vertices[0].y };
            const glm::vec3 &v1f = { t.vertices[1].x, t.vertices[1].z, t.vertices[1].y };
            const glm::vec3 &v2f = { t.vertices[2].x, t.vertices[2].z, t.vertices[2].y };

            return { t.id, v0f, v2f, v1f, t.material };
        });
    }

    // camera
    const float cx = std::cos(2.0f) * 2.0f;
    const float cz = std::sin(2.0f) * 2.0f;

    Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};

    const auto start_bvh = steady_clock::now();

#if ENABLE_OBB_BVH
    core::obb::ObbTree obbTree(triangles, ENABLE_OBB_SAH, ENABLE_CLUSTERING, NUM_CLUSTERS);
    if (obbTree.failed())
    {
        std::cerr << "ObbTree construction failed" << std::endl;
        return EXIT_FAILURE;
    }
#elif ENABLE_HYBRID_BVH
    BVH bvh(triangles, ENABLE_HYBRID_BVH, 0);
    if (bvh.failed())
    {
        std::cerr << "BVH construction failed" << std::endl;
        return EXIT_FAILURE;
    }
#else
    BVH bvh(triangles, ENABLE_AABB_WITH_OBB);
    if (bvh.failed())
    {
        std::cerr << "BVH construction failed" << std::endl;
        return EXIT_FAILURE;
    }
#endif

    // Bounding box visualization
#if ENABLE_OBB_BVH
    #if ENABLE_CLUSTERING
        debug::Visualization visualization(obbTree);
        visualization.visualizationClustering(obbTree.getClusterOBBs());
        std::vector<core::Triangle> temTris = visualization.getTriangles();
        BVH boundingBoxBVH(temTris);
    #else
        debug::Visualization visualization(obbTree);
        BVH boundingBoxBVH(visualization.getTriangles(), false);
    #endif
#else
    debug::Visualization visualization(bvh);
    BVH boundingBoxBVH(visualization.getTriangles(), false);
#endif

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

    float max_rps = -INF;
    float min_rps = INF;

    // For debugging BVH nodes
#if ENABLE_OBB_BVH
    int maxDepth = obbTree.getMaxDepth();
    int nodeCount = obbTree.getNodes().size();
#else
    int maxDepth = bvh.getMaxDepth();
    int nodeCount = bvh.getNodes().size();
#endif

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

        // Render frame
        const auto start = steady_clock::now();

        // not use SIMD for now
        #if 1
            #if ENABLE_OBB_BVH
                if (GlobalState::bboxView)
                {
                    render_frame_4x4(camera, boundingBoxBVH, frame.pixels.get());
                }
                else
                {
                    render_frameOBB(camera, obbTree, frame.pixels.get(), ENABLE_CLUSTERING, ENABLE_CACHING);
                }
            #elif ENABLE_HYBRID_BVH
                if (GlobalState::bboxView)
                {
                    render_frame_4x4(camera, boundingBoxBVH, frame.pixels.get());
                }
                else
                {
                    render_frameHybrid(camera, bvh, frame.pixels.get(), ENABLE_CACHING);
                }

            #else
                if (GlobalState::bboxView)
                {
                    render_frame_4x4(camera, boundingBoxBVH, frame.pixels.get());
                }
                else
                {
                    render_frame(camera, bvh, frame.pixels.get(), ENABLE_AABB_WITH_OBB, ENABLE_CACHING);
                }
            #endif
        #else
            render_frame_4x4(camera, bvh, frame.pixels.get());
        #endif

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

                ImGui::Text("Number of Nodes: %d", nodeCount);
                ImGui::Text("Max Depth: %d", maxDepth);
                ImGui::Separator();
                if (ImGui::CollapsingHeader("BVH Nodes", ImGuiTreeNodeFlags_DefaultOpen))
                {
#if ENABLE_OBB_BVH

#else
                    debug::renderBVHtree(bvh.getRoot(), bvh.getNodes());
#endif
                }

                ImGui::Separator();

                if (ImGui::Checkbox("Ray heatmap view", &GlobalState::heatmapView))
                {
                    GlobalState::bboxView = false;
                }
                if (ImGui::Checkbox("BVH bounding box view", &GlobalState::bboxView))
                {
                    GlobalState::heatmapView = false;
                }
            }

            ImGui::End();
        }

        ImGui::Render();

        ImGui_ImplSDLRenderer_RenderDrawData(ImGui::GetDrawData());

        SDL_RenderPresent(renderer);
        FrameMark;
    }

    return EXIT_SUCCESS;
}
