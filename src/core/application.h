#pragma once

#include <string>
#include <vector>

#include "utils/window.h"
#include "core/types.h"
#include "utils/globalState.h"
#include "utils/obj.h"
#include "core/trace.h"
#include "utils/settings.h"

namespace core
{
    class Application
    {
        using Model = std::vector<Triangle>;
    public:
        Application();

        void init();
        void run();

    private:
        // Window
        utils::Window m_window;

        // Model
        std::string m_modelPath;
        std::vector<Model> m_models;
        Model m_mesh;
        bool m_flip;

        // tracer
        Tracer m_tracer;

    };
}