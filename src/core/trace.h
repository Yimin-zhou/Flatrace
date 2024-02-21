#pragma once

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

constexpr int WINDOW_WIDTH = 1280;
constexpr int WINDOW_HEIGHT = 720;

constexpr int FRAME_WIDTH = 1280;
constexpr int FRAME_HEIGHT = 720;

constexpr auto VIEWPORT_WIDTH  = 2.4f;
constexpr auto VIEWPORT_HEIGHT = 1.35f;

constexpr auto DX = VIEWPORT_WIDTH / FRAME_WIDTH;
constexpr auto DY = VIEWPORT_HEIGHT / FRAME_HEIGHT;

constexpr auto TILE_SIZE = 16;
constexpr auto BUNDLE_SIZE = 4;

constexpr auto NX = FRAME_WIDTH / TILE_SIZE;
constexpr auto NY = FRAME_HEIGHT / TILE_SIZE;

constexpr auto N_FRAMES = 1;
constexpr auto N_RAYS = N_FRAMES * FRAME_WIDTH * FRAME_HEIGHT;

constexpr auto MAX_INTERSECTIONS = 3;

constexpr auto SPEED = 0.0f;

void render_frame(const core::Camera &camera, const core::BVH &bvh, core::RGBA * const frameBuffer, int maxDepth);
void render_frame_4x4(const core::Camera &camera, const core::BVH &bvh, core::RGBA * const frameBuffer);
