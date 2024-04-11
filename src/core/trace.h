#pragma once

#include "src/core/bvh/obbTree.h"
#include "src/core/types.h"
#include "src/core/frame.h"
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

//#include "utils/settings.h"
namespace core
{
    class Tracer
    {
    public:
        Tracer() = default;

        Tracer(const std::vector<std::vector<Triangle>> &meshes,
               int width, int height, int maxIterations, float viewWidth,
               float viewHeight, int tileSize, int bundleSize);

        void resize(int width, int height);
        void render(const core::Camera &camera);

        RGBA* getPixels() { return m_frame.pixels.get(); }

    private:
        std::vector<std::vector<Triangle>> m_meshes;
#if GEN_OBB_BVH
        core::ObbTree m_bvh;
#else
        BVH m_bvh;
#endif
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

        // debugs
        debug::Visualization m_visualization;
        core::BVH m_bboxBVH;

        int _sampleRate = 1000;
        int _rayCount = 0;

        void renderFrame(const BVH& bvh, const core::Camera &camera);

        void renderFrame4X4(const BVH& bvh, const core::Camera &camera);

        void renderFrameObb(const BVH& bvh, const core::Camera &camera, const core::ObbTree &obbTree,
                            core::RGBA *const frameBuffer,
                            int maxDepth);

//        float rayProcessingTimes[N_RAYS];
        std::array<std::array<float, 4>, 8> getMaterial();
        glm::vec3 getColorMap(int value, int minVal, int maxVal);
    };
}