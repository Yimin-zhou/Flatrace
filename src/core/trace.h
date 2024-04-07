#pragma once

#include "src/core/obbTree.h"
#include "src/core/types.h"
#include "src/core/frame.h"
#include "src/core/bvh.h"
#include <tbb/parallel_for.h>
#include "src/utils/globalState.h"
#include "src/debug/visualization.h"
#include <SDL2/SDL.h>
#include <imgui/imgui.h>
#include <imgui/imgui_impl_sdl.h>
#include <imgui/imgui_impl_sdlrenderer.h>
#include <fmt/format.h>
#include <chrono>
#include <iostream>

//#include "utils/settings.h"
namespace core
{
    class Tracer
    {
    public:
        Tracer(const std::vector<Triangle>& mesh, int width, int height, int maxIterations,
               float viewWidth, float viewHeight, int tileSize, int bundleSize);
        Tracer() = default;

        void resize(int width, int height);
        void render(const core::Camera &camera);

        RGBA* getPixels() { return m_frame.pixels.get(); }

    private:
        std::vector<Triangle> m_mesh;

        BVH m_bvh;
        Frame m_frame;

        int m_width;
        int m_height;
        float m_viewportWidth;
        float m_viewportHeight;
        int m_maxIterations;
        // rays
        int m_tileSize;
        int m_bundleSize;
        float m_nx, m_ny;
        float m_dx, m_dy;

        int _sampleRate = 1000;
        int _rayCount = 0;

        void renderFrame(const core::Camera &camera);

        void renderFrame4X4(const core::Camera &camera);

        void renderFrameObb(const core::Camera &camera, const core::ObbTree &obbTree,
                            core::RGBA *const frameBuffer,
                            int maxDepth);

//        float rayProcessingTimes[N_RAYS];
        std::array<std::array<float, 4>, 8> getMaterial();
        glm::vec3 getColorMap(int value, int minVal, int maxVal);
    };
}