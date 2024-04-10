#pragma once

#include "core/obbTree.h"
#include "core/types.h"
#include "core/bvh.h"
#include <tbb/parallel_for.h>
#include "utils/globalState.h"
#include "debug/visualization.h"
#include <SDL2/SDL.h>
#include <imgui/imgui.h>
#include <imgui/imgui_impl_sdl.h>
#include <imgui/imgui_impl_sdlrenderer.h>
#include <fmt/format.h>
#include <chrono>
#include <iostream>
#include "core/frame.h"
#include "imguiManager.h"

namespace utils
{
    class Window
    {
    public:
        Window(const std::string &title, int width, int height);
        ~Window();

        void resize();
        void display(core::RGBA* pixels);

        int getWidth() const { return m_windowWidth; }
        int getHeight() const { return m_windowHeight; }

        SDL_Renderer* getRenderer() const { return m_renderer; }
        SDL_Window* getWindow() const { return m_window; }

    private:
        // render texture
        SDL_Texture* m_texture = nullptr;

        SDL_Window* m_window = nullptr;
        SDL_Renderer* m_renderer = nullptr;
        int m_windowWidth;
        int m_windowHeight;

        // imgui
        ImGuiManager m_imguiManager;
    };
}
