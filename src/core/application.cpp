#include "application.h"
#include <Tracy.hpp>

namespace core
{
    Application::Application() :
            m_window("Flatrace", WINDOW_WIDTH, WINDOW_HEIGHT),
            m_modelPath("test/input/test"),
            m_flip(false)
    {
        init();
    }

    void Application::init()
    {
        // Load objects from folder
        std::vector<Model> models;
        models = utils::Obj::loadAllObjFilesInFolder(m_modelPath, MODEL_NORMALIZE);
//        Model test_model = utils::Obj::read("test/input/test/bunny.obj", true);
//        models[0] = test_model;

        m_tracer = Tracer(models, WINDOW_WIDTH, WINDOW_HEIGHT, MAX_INTERSECTIONS,
                          VIEWPORT_WIDTH, VIEWPORT_HEIGHT, m_tileSize, m_bundleSize, ENABLE_OBB_BVH);

    }

    void Application::run()
    {
        bool quit = false;
        while (!quit)
        {
            // handle input
            SDL_Event e;
            while (SDL_PollEvent(&e))
            {
//                ImGui_ImplSDL2_ProcessEvent(&e);
                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
                else if (e.type == SDL_WINDOWEVENT && e.window.event == SDL_WINDOWEVENT_RESIZED)
                {
                    m_window.resize();
                    m_tracer.resize(m_window.getWidth(), m_window.getHeight());
                }
            }

            float theta = 2.0f;
            // Update camera
            const float cx = std::cos(theta) * 2.0f;
            const float cz = std::sin(theta) * 2.0f;

            Camera camera = {{cx, 1.0f, cz}, {0.0f, 0.0f, 0.0f},
                             {0.0f, 1.0f, 0.0f}, 5.0f};

            // Render frame
            m_tracer.render(camera, ENABLE_OBB_TRACING);

            // Display frame
            m_window.display(m_tracer.getPixels());
            FrameMark;
        }
    }
}
