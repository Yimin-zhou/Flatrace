#pragma once

#include "src/core/bvh/obbTree.h"
#include "src/core/bvh/aabbTree.h"
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
               float viewHeight, int tileSize, int bundleSize, bool genObbBvh);

        void resize(int width, int height);
        void render(const core::Camera &camera, bool traverseObbInAabb);

        Frame &getFrame() { return m_frame; }
        RGBA* getPixels() { return m_frame.pixels.get(); }

    private:
        std::shared_ptr<BVH> m_bvh;
        std::vector<std::vector<Triangle>> m_meshes;

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
        std::shared_ptr<BVH> m_bboxBVH;

        int _sampleRate = 1000;
        int _rayCount = 0;

        void renderFrame(const core::Camera &camera, bool traverseObb = false);
        void renderBboxFrame(const core::Camera &camera, bool traverseObb = false);

        void renderFrame4X4(const core::Camera &camera, bool traverseObb = false);

//        float rayProcessingTimes[N_RAYS];
        std::array<std::array<float, 4>, 8> getMaterial();
        glm::vec3 getColorMap(int value, int minVal, int maxVal);
    };
}