#pragma once

#include "core/types.h"

#include <imgui/imgui.h>
#include <imgui/imgui_impl_sdl.h>
#include <imgui/imgui_impl_sdlrenderer.h>

namespace utils
{
    class ImGuiManager
    {
    public:
        ImGuiManager(SDL_Window* window, SDL_Renderer* renderer);
        ImGuiManager() = default;

        void draw();

    };
}