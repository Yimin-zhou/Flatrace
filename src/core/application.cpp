#include "application.h"

namespace core
{
    Application::Application() :
            m_window("Flatrace", m_width, m_height),
            m_modelPath("test/input/big_obj"),
            m_flip(false)
    {
        init();
    }

    void Application::init()
    {
        // Load objects from folder
        std::vector<Model> models;
        models = utils::Obj::loadAllObjFilesInFolder("test/input/big_obj", false);

        // add all triangles from all objects
        for (int i = 0; i < models.size(); i++)
        {
            m_mesh.insert(m_mesh.end(), models[i].begin(), models[i].end());
        }

        if (m_flip)
        {
            std::transform(m_mesh.begin(), m_mesh.end(), m_mesh.begin(), [](const Triangle &t) -> Triangle
            {
                const glm::vec3 &v0f = {t.vertices[0].x, t.vertices[0].z, t.vertices[0].y};
                const glm::vec3 &v1f = {t.vertices[1].x, t.vertices[1].z, t.vertices[1].y};
                const glm::vec3 &v2f = {t.vertices[2].x, t.vertices[2].z, t.vertices[2].y};

                return {t.id, v0f, v2f, v1f, t.material};
            });
        }

        m_tracer = Tracer(m_mesh, m_width, m_height, MAX_INTERSECTIONS,
                          VIEWPORT_WIDTH, VIEWPORT_HEIGHT, m_tileSize, m_bundleSize);
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
//        ImGui_ImplSDL2_ProcessEvent(&e);
                if (e.type == SDL_QUIT)
                {
                    quit = true;
                } else if (e.type == SDL_WINDOWEVENT && e.window.event == SDL_WINDOWEVENT_RESIZED) {
                    m_window.resize();
                    m_tracer.resize(m_window.getWidth(), m_window.getHeight());
                }
            }

            float theta = 2.0f;
            // Update camera
            const float cx = std::cos(theta) * 2.0f;
            const float cz = std::sin(theta) * 2.0f;

            Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz},
                             {0.0f, 1.0f, 0.0f}, 5.0f};

            // Render frame
            m_tracer.render(camera);

            // Display frame
            m_window.display(m_tracer.getPixels());
        }
    }
}
