#include "window.h"

utils::Window::Window(const std::string &title, int width, int height) : m_windowWidth(width), m_windowHeight(height)
{
    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        throw std::runtime_error("Failed to initialize SDL2: " + std::string(SDL_GetError()));
    }

    m_window = SDL_CreateWindow(title.c_str(), SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width, height, SDL_WINDOW_ALLOW_HIGHDPI | SDL_WINDOW_RESIZABLE);
    if (!m_window)
    {
        throw std::runtime_error("Failed to create window: " + std::string(SDL_GetError()));
    }

    m_renderer = SDL_CreateRenderer(m_window, -1, SDL_RENDERER_PRESENTVSYNC);
    if (!m_renderer)
    {
        throw std::runtime_error("Failed to create renderer: " + std::string(SDL_GetError()));
    }

    m_texture = SDL_CreateTexture(m_renderer, SDL_PIXELFORMAT_ABGR8888, SDL_TEXTUREACCESS_STREAMING, width, height);
    if (!m_texture)
    {
        throw std::runtime_error("Failed to create texture: " + std::string(SDL_GetError()));
    }

    SDL_RenderSetVSync(m_renderer, 1);
    resize();

    // imgui
    m_imguiManager = ImGuiManager(m_window, m_renderer);
}

utils::Window::~Window()
{
    if (m_renderer) SDL_DestroyRenderer(m_renderer);
    if (m_window) SDL_DestroyWindow(m_window);
    SDL_Quit();
}

void utils::Window::resize()
{
    int windowWidth, windowHeight;
    SDL_GetWindowSize(m_window, &windowWidth, &windowHeight);
    int targetHeight = windowWidth * 9 / 16;
    if (targetHeight > windowHeight)
    {
        windowWidth = windowHeight * 16 / 9;
    }
    else
    {
        windowHeight = targetHeight;
    }
  // maintain aspect ratio
  SDL_RenderSetLogicalSize(m_renderer, windowWidth, windowHeight);

    int drawableWidth, drawableHeight;
    SDL_GetRendererOutputSize(m_renderer, &drawableWidth, &drawableHeight);
    float scaleX = (float)drawableWidth / windowWidth;
    float scaleY = (float)drawableHeight / windowHeight;
    SDL_RenderSetScale(m_renderer, scaleX, scaleY);
    // rescale the texture
    SDL_DestroyTexture(m_texture);
    m_texture = SDL_CreateTexture(m_renderer, SDL_PIXELFORMAT_ABGR8888, SDL_TEXTUREACCESS_STREAMING, windowWidth, windowHeight);
    m_windowWidth = windowWidth;
    m_windowHeight = windowHeight;
}

void utils::Window::display(core::RGBA* pixels)
{
    SDL_UpdateTexture(m_texture, NULL, pixels, m_windowWidth * sizeof(core::RGBA));
    SDL_RenderCopy(m_renderer, m_texture, NULL, NULL);

    // imgui
    m_imguiManager.draw();

    SDL_RenderPresent(m_renderer);
}




