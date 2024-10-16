#pragma once

#include "src/core/types.h"
#include "src/core/frame.h"
#include "src/core/bvh.h"
#include "src/core/obbTree.h"
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



// AABB
void render_frame(const core::Camera &camera, core::BVH &bvh, core::RGBA *const frameBuffer, bool obbInAABBbvh);

void render_frame_4x4(const core::Camera &camera, const core::BVH &bvh, core::RGBA *const frameBuffer);

// OBB
void cacheRayDirs(core::obb::ObbTree &obbTree, std::vector<glm::vec3> &out, const glm::vec3 &rayDir);

void
render_frameOBB(const core::Camera &camera, core::obb::ObbTree &obb, core::RGBA *const frameBuffer);

void render_frame_4x4OBB(const core::Camera &camera, core::obb::ObbTree &obbTree, core::RGBA *const frameBuffer);

// HybriduseCaching
void render_frameHybrid(const core::Camera &camera, core::BVH &bvh, core::RGBA *const frameBuffer);

