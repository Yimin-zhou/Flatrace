#pragma once

#include "src/core/types.h"
#include "src/core/frame.h"
#include "src/core/bvh.h"

#include "src/utils/ppm.h"
#include "src/utils/obj.h"

#include "src/debug/visualization.h"

#include <SDL2/SDL.h>

#include <tbb/parallel_for.h>

#include <imgui/imgui.h>
#include <imgui/imgui_impl_sdl.h>
#include <imgui/imgui_impl_sdlrenderer.h>

#include <fmt/format.h>

#include <chrono>
#include <iostream>

namespace {
    constexpr auto WINDOW_WIDTH = 1024;
    constexpr auto WINDOW_HEIGHT = 768;

    constexpr auto FRAME_WIDTH = 1024;
    constexpr auto FRAME_HEIGHT = 768;

    constexpr auto VIEWPORT_WIDTH  = 1.2f;
    constexpr auto VIEWPORT_HEIGHT = 1.2f;

    constexpr auto DX = VIEWPORT_WIDTH / FRAME_WIDTH;
    constexpr auto DY = VIEWPORT_HEIGHT / FRAME_HEIGHT;

    constexpr auto TILE_SIZE = 16;
    constexpr auto BUNDLE_SIZE = 4;

    constexpr auto NX = FRAME_WIDTH / TILE_SIZE;
    constexpr auto NY = FRAME_HEIGHT / TILE_SIZE;

    constexpr auto N_FRAMES = 1;
    constexpr auto N_RAYS = N_FRAMES * FRAME_WIDTH * FRAME_HEIGHT;

    constexpr auto MAX_INTERSECTIONS = 3;

    constexpr auto SPEED = 0.05f;

    // Arbitrary color palette for materials
    static const std::array<std::array<float, 4>, 8> COLORS = { {
                                                                        { { 1.0f, 0.0f, 0.0f, 1.0f } },
                                                                        { { 0.0f, 1.0f, 0.0f, 1.0f } },
                                                                        { { 0.0f, 0.0f, 1.0f, 1.0f } },
                                                                        { { 1.0f, 1.0f, 0.0f, 1.0f } },
                                                                        { { 1.0f, 0.0f, 1.0f, 1.0f } },
                                                                        { { 0.0f, 1.0f, 1.0f, 1.0f } },
                                                                        { { 1.0f, 0.5f, 0.5f, 1.0f } },
                                                                        { { 0.5f, 0.5f, 1.0f, 1.0f } },
                                                                } };

    // For debugging
    core::Vec3 get_color_map(int value, int minVal, int maxVal)
    {
        float normalized = static_cast<float>(value - minVal) / static_cast<float>(maxVal - minVal);

        //from blue to red.
        core::Vec3 blue = {0.0f, 0.0f, 1.0f};
        core::Vec3 red = {1.0f, 0.0f, 0.0f};

        core::Vec3 color;
        color = blue + (red - blue) * normalized;

        return color;
    }


    // Reference implementation that traces 1 ray at a time (no SIMD)
    void render_frame(const core::Camera &camera, const core::BVH &bvh, core::RGBA * const frameBuffer)
    {
        tbb::parallel_for(tbb::blocked_range<int>(0, NX*NY), [&](const tbb::blocked_range<int> &r)
        {
            for (int tile_idx = r.begin(); tile_idx != r.end(); tile_idx++)
            {
                const int tile_i = (tile_idx / NX);
                const int tile_j = (tile_idx % NX);

                const float tile_y = -(VIEWPORT_HEIGHT / 2.0f) + (tile_i * TILE_SIZE * DY);
                const float tile_x = -(VIEWPORT_WIDTH / 2.0f) + (tile_j * TILE_SIZE * DX);

                float y = tile_y;
                core::RGBA * p = frameBuffer + (FRAME_HEIGHT - tile_i*TILE_SIZE - 1)*FRAME_WIDTH + (tile_j*TILE_SIZE);
                for (int i = tile_i*TILE_SIZE; i < (tile_i*TILE_SIZE) + TILE_SIZE; i++)
                {
                    float x = tile_x;

                    for (int j = tile_j*TILE_SIZE; j < (tile_j*TILE_SIZE) + TILE_SIZE; j++)
                    {
                        const core::Vec3 ray_origin = camera.p + camera.x*x + camera.y*y;
                        const core::Vec3 ray_direction = camera.d;

                        core::Ray ray = { ray_origin, ray_direction };

                        const bool hit = bvh.intersect(ray, MAX_INTERSECTIONS);

                        const float src_alpha = 1.0f;

                        __m128i c = _mm_set1_epi32(0);

                        if (hit)
                        {
                            __m128 cf = _mm_set1_ps(0.0f);

                            float dst_alpha = 1.0f;

                            for (int n = 0; n < 3; n++)
                            {
                                const int triangle = ray.triangle[n];

                                // c += COLORS[bvh.getTriangle(ray.triangle[0]).material & 0x07] * (dst_alpha * src_alpha * std::abs(ray.dot[n]));
                                const __m128 abs_dot_x4 = _mm_andnot_ps(_mm_set1_ps(-0.0f), _mm_load1_ps(&ray.dot[n]));
                                const __m128 tri_color = _mm_load_ps(COLORS[bvh.getTriangle(triangle).material & 0x07].data());
                                const __m128 shaded_color = _mm_mul_ps(tri_color, abs_dot_x4);

                                const float alpha = dst_alpha * src_alpha;

                                cf = _mm_add_ps(cf, _mm_mul_ps(_mm_load1_ps(&alpha), shaded_color));

                                dst_alpha *= (1.0 - src_alpha);
                            }
                            // c = min(255 * cf)
                            cf = _mm_min_ps(_mm_mul_ps(cf, _mm_set1_ps(255.0f)), _mm_set1_ps(255.0f));
                            c = _mm_shuffle_epi8(_mm_cvtps_epi32(cf), _mm_set1_epi32(0x0C080400));

#ifdef DEBUG
                            {
                                core::Vec3 heat_map_color = get_color_map(
                                        ray.bvh_nodes_visited, 0, 100);
                                cf = _mm_set_ps(heat_map_color.z, heat_map_color.y, heat_map_color.x, 1.0f);
                                cf = _mm_min_ps(_mm_mul_ps(cf, _mm_set1_ps(255.0f)), _mm_set1_ps(255.0f));
                                c = _mm_shuffle_epi8(_mm_cvtps_epi32(cf), _mm_set1_epi32(0x0C080400));
                            }
#endif
                        }

                        // *p = c;
                        _mm_storeu_si32(p, c);

                        x += DX;
                        p += 1;
                    }

                    y += DY;
                    p -= (FRAME_WIDTH + TILE_SIZE);
                }
            }
        });
    }

    // 8-way SIMD implementation that traces 4x4 'ray bundles'
    void render_frame_4x4(const core::Camera &camera, const core::BVH &bvh, core::RGBA * const frameBuffer)
    {
        const core::Vec3 rd = { 1.0f / camera.d.x, 1.0f / camera.d.y, 1.0f / camera.d.z };

        tbb::parallel_for(tbb::blocked_range<int>(0, NX*NY), [&](const tbb::blocked_range<int> &r)
        {
            for (int tile_idx = r.begin(); tile_idx != r.end(); tile_idx++)
            {
                const int tile_i = (tile_idx / NX);
                const int tile_j = (tile_idx % NX);

                for (int bundle_i = 0; bundle_i < TILE_SIZE/BUNDLE_SIZE; bundle_i++)
                {
                    const int bundle_py = (tile_i * TILE_SIZE) + (bundle_i * BUNDLE_SIZE);
                    const float bundle_y = -(VIEWPORT_HEIGHT / 2.0f) + (bundle_py * DY);

                    for (int bundle_j = 0; bundle_j < TILE_SIZE/BUNDLE_SIZE; bundle_j++)
                    {
                        const int bundle_px = (tile_j * TILE_SIZE) + (bundle_j * BUNDLE_SIZE);
                        const float bundle_x = -(VIEWPORT_WIDTH / 2.0f) + (bundle_px * DX);

                        core::RGBA * const p = frameBuffer + ((FRAME_HEIGHT - bundle_py - BUNDLE_SIZE) * FRAME_WIDTH) + bundle_px;

                        const core::Vec3 bundle_origin = camera.p + camera.x*bundle_x + camera.y*bundle_y;

                        core::Ray4x4 rays = { camera, bundle_origin, camera.d, rd, DX, DY };

                        const bool hit = bvh.intersect4x4(rays, MAX_INTERSECTIONS);

                        __m128 src_alpha = _mm_set1_ps(1.0f);

                        for (int r = 0; r < 16; r++)
                        {
                            const int ray_i = r / 4;
                            const int ray_j = r % 4;

                            __m128i c = _mm_set1_epi32(0);

                            if (hit)
                            {
                                __m128 dst_alpha = _mm_set1_ps(1.0f);

                                __m128 cf = _mm_set1_ps(0.0f);

                                for (int n = 0; n < 3; n++)
                                {
                                    const int triangle = rays.triangle[n*16 + r];

                                    // c += COLORS[bvh.getTriangle(ray.triangle[0]).material & 0x07] * (dst_alpha * src_alpha * std::abs(ray.dot[n]));
                                    const __m128 abs_dot_x4 = _mm_andnot_ps(_mm_set1_ps(-0.0f), _mm_load1_ps(rays.dot.data() + n*16 + r));
                                    const __m128 tri_color = _mm_load_ps(COLORS[bvh.getTriangle(triangle).material & 0x07].data());
                                    const __m128 shaded_color = _mm_mul_ps(tri_color, abs_dot_x4);

                                    const __m128 alpha = _mm_mul_ps(dst_alpha, src_alpha);

                                    cf = _mm_add_ps(cf, _mm_mul_ps(alpha, shaded_color));

                                    // dst_alpha *= (1.0 - src_alpha);
                                    dst_alpha = _mm_mul_ps(dst_alpha, _mm_sub_ps(_mm_set1_ps(1.0f), src_alpha));
                                }

//              // For debugging: Retrieve the number of nodes this ray has traversed.
//              {
//                    int nodesTraversed = rays.bvh_nodes_visited[r];  // assuming nodes_visited is an array of 16 ints
//                    // Modify color based on the number of traversed nodes.
//                    float factor = static_cast<float>(nodesTraversed) / 1.0f;  // Convert to a suitable factor, e.g., [0.0, 1.0]
//                    factor = (factor > 1.0f) ? 1.0f : factor;  // clamp the value
//                    __m128 factor_vec = _mm_set1_ps(factor);
//                    __m128 red_channel = _mm_shuffle_ps(cf, cf, _MM_SHUFFLE(0, 0, 0, 0)); // isolate red channel
//                    // Apply the factor only to the red channel
//                    red_channel = _mm_mul_ps(red_channel, factor_vec);
//
//                    // Combine back into the original color
//                    __m128 green_blue_alpha = _mm_shuffle_ps(cf, cf,
//                                                             _MM_SHUFFLE(3, 2, 1, 1)); // keep green, blue, and alpha
//                    cf = _mm_move_ss(green_blue_alpha, red_channel); // replace red with modified value
//              }
                                // c = min(255 * cf)
                                cf = _mm_min_ps(_mm_mul_ps(cf, _mm_set1_ps(255.0f)), _mm_set1_ps(255.0f));
                                c = _mm_shuffle_epi8(_mm_cvtps_epi32(cf), _mm_set1_epi32(0x0C080400));
                            }

                            _mm_storeu_si32(p + ((BUNDLE_SIZE - ray_i - 1) * FRAME_WIDTH) + ray_j, c);
                        }
                    }
                }
            }
        });
    }

}