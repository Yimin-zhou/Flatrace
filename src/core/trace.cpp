#include "trace.h"
#include "src/utils/globalState.h"
#include "src/utils/globalState.h"
#include "Tracy.hpp"

// Arbitrary color palette for materials
auto getMaterial()
{
    std::array<std::array<float, 4>, 8> COLORS;
    if (DebugState::BBOX_VIEW)
    {
        COLORS = {{
                          {{1.0f, 0.5f, 0.5f, 1.0f}},
                          {{0.0f, 1.0f, 0.0f, 1.0f}},
                          {{0.0f, 0.0f, 1.0f, 1.0f}},
                          {{1.0f, 1.0f, 0.0f, 1.0f}},
                          {{1.0f, 0.0f, 1.0f, 1.0f}},
                          {{0.0f, 1.0f, 1.0f, 1.0f}},
                          {{1.0f, 0.5f, 0.5f, 1.0f}},
                          {{0.5f, 0.5f, 1.0f, 1.0f}},
                  }};
    } else
    {
        COLORS = {{
                          {{1.0f, 1.0f, 1.0f, 1.0f}},
                          {{0.0f, 1.0f, 0.0f, 1.0f}},
                          {{0.0f, 0.0f, 1.0f, 1.0f}},
                          {{1.0f, 1.0f, 0.0f, 1.0f}},
                          {{1.0f, 0.0f, 1.0f, 1.0f}},
                          {{0.0f, 1.0f, 1.0f, 1.0f}},
                          {{1.0f, 0.5f, 0.5f, 1.0f}},
                          {{0.5f, 0.5f, 1.0f, 1.0f}},
                  }};
    }
    return COLORS;
}

// For debugging
glm::vec3 getColorMap(int value, int minVal, int maxVal)
{
    // create a gradient from blue -> green -> red, and use it as a lookup table
    std::vector<glm::vec3> color_map =
            {
                    {0.0f, 0.0f, 1.0f}, // blue
                    {0.0f, 1.0f, 1.0f}, // cyan
                    {0.0f, 1.0f, 0.0f}, // green
                    {1.0f, 1.0f, 0.0f}, // yellow
                    {1.0f, 0.0f, 0.0f}, // red
            };

    float factor = static_cast<float>(value - minVal) / static_cast<float>(maxVal - minVal);
    factor = (factor > 1.0f) ? 1.0f : factor;
    factor = (factor < 0.0f) ? 0.0f : factor;

    // linearly interpolate between the two colors
    float f_index = factor * (color_map.size() - 1);
    int index = static_cast<int>(f_index);
    float fraction = f_index - index;

    glm::vec3 color = color_map[index] * (1.0f - fraction) + color_map[index + 1] * fraction;
    return color;
}

// For AABB tree
void cacheRayDirsAABBTree(core::BVH &bvh, std::vector<glm::vec3>& out, const glm::vec3 &rayDir, bool useClustering)
{
    for (auto &node: bvh.getNodes())
    {
        if (node.obbFlag && useClustering)
        {
            std::vector<glm::mat4x4> cachedObbTransformations = bvh.getTransformationCache();
            for (auto &obbTransformation: cachedObbTransformations)
            {
                out.emplace_back(obbTransformation * glm::vec4(rayDir, 0.0f));
            }
        }
        else
        {
            node.cachedRayDir = glm::vec3((node.obb.invMatrix) * glm::vec4(rayDir, 0.0f));
        }
    }
}

// Reference implementation that traces 1 ray at a time (no SIMD)
void render_frame(const core::Camera &camera, core::BVH &bvh, core::RGBA *const frameBuffer, bool obbInAABBbvh,
                  bool useCaching, bool useClustering)
{
    const glm::vec3 ray_direction = camera.dir;
    std::vector<glm::vec3> cachedRayDirs;

    if (useCaching)
    {
        cacheRayDirsAABBTree(bvh, cachedRayDirs, ray_direction, useClustering);
    }

    auto COLORS = getMaterial();
    tbb::parallel_for(tbb::blocked_range<int>(0, NX * NY), [&](const tbb::blocked_range<int> &r)
    {
        for (int tile_idx = r.begin(); tile_idx != r.end(); tile_idx++)
        {
            const int tile_i = (tile_idx / NX);
            const int tile_j = (tile_idx % NX);

            const float tile_y = -(VIEWPORT_HEIGHT / 2.0f) + (tile_i * TILE_SIZE * DY);
            const float tile_x = -(VIEWPORT_WIDTH / 2.0f) + (tile_j * TILE_SIZE * DX);

            float y = tile_y;
            core::RGBA *p = frameBuffer + (FRAME_HEIGHT - tile_i * TILE_SIZE - 1) * FRAME_WIDTH + (tile_j * TILE_SIZE);
            for (int i = tile_i * TILE_SIZE; i < (tile_i * TILE_SIZE) + TILE_SIZE; i++)
            {
                float x = tile_x;

                for (int j = tile_j * TILE_SIZE; j < (tile_j * TILE_SIZE) + TILE_SIZE; j++)
                {
                    const glm::vec3 ray_origin = camera.pos + camera.x * x + camera.y * y;

                    core::Ray ray = {ray_origin, ray_direction};

                    bool hit = false;

                    if (obbInAABBbvh)
                    {
                        hit = bvh.traversalOBB(ray, MAX_INTERSECTIONS, cachedRayDirs, useCaching);
                    } else
                    {
                        hit = bvh.traversal(ray, MAX_INTERSECTIONS);
                    }

                    const float src_alpha = 0.4f;

                    __m128i c = _mm_set1_epi32(0);

                    if (hit)
                    {
                        __m128 cf = _mm_set1_ps(0.0f);

                        if (DebugState::HEATMAP_VIEW)
                        {
                            glm::vec3 heat_map_color = getColorMap(ray.bvh_nodes_visited, 1, 200);
                            cf = _mm_set_ps(1.0f, heat_map_color.z, heat_map_color.y, heat_map_color.x);
                            cf = _mm_min_ps(_mm_mul_ps(cf, _mm_set1_ps(255.0f)), _mm_set1_ps(255.0f));
                            c = _mm_shuffle_epi8(_mm_cvtps_epi32(cf), _mm_set1_epi32(0x0C080400));
                        } else
                        {
                            float dst_alpha = 1.0f;

                            for (int n = 0; n < 3; n++)
                            {
                                const int triangle = ray.triangle[n];

                                // c += COLORS[bvh.getTriangle(ray.triangle[0]).material & 0x07] * (dst_alpha * src_alpha * std::abs(ray.dot[n]));
                                const __m128 abs_dot_x4 = _mm_andnot_ps(_mm_set1_ps(-0.0f),
                                                                        _mm_load1_ps(&ray.dot[n]));
                                const __m128 tri_color = _mm_load_ps(
                                        COLORS[bvh.getTriangle(triangle).material & 0x07].data());
                                const __m128 shaded_color = _mm_mul_ps(tri_color, abs_dot_x4);

                                const float alpha = dst_alpha * src_alpha;

                                cf = _mm_add_ps(cf, _mm_mul_ps(_mm_load1_ps(&alpha), shaded_color));

                                dst_alpha *= (1.0 - src_alpha);
                            }
                            // c = min(255 * cf)
                            cf = _mm_min_ps(_mm_mul_ps(cf, _mm_set1_ps(255.0f)), _mm_set1_ps(255.0f));
                            c = _mm_shuffle_epi8(_mm_cvtps_epi32(cf), _mm_set1_epi32(0x0C080400));
                        }
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

void render_frameHybrid(const core::Camera &camera, core::BVH &bvh, core::RGBA *const frameBuffer, bool useCaching, bool useClustering)
{
    const glm::vec3 ray_direction = camera.dir;
    std::vector<glm::vec3> cachedRayDirs;

    if (useCaching)
    {
        cacheRayDirsAABBTree(bvh, cachedRayDirs, ray_direction, useClustering);
    }

    auto COLORS = getMaterial();
    tbb::parallel_for(tbb::blocked_range<int>(0, NX * NY), [&](const tbb::blocked_range<int> &r)
    {
        for (int tile_idx = r.begin(); tile_idx != r.end(); tile_idx++)
        {
            const int tile_i = (tile_idx / NX);
            const int tile_j = (tile_idx % NX);

            const float tile_y = -(VIEWPORT_HEIGHT / 2.0f) + (tile_i * TILE_SIZE * DY);
            const float tile_x = -(VIEWPORT_WIDTH / 2.0f) + (tile_j * TILE_SIZE * DX);

            float y = tile_y;
            core::RGBA *p = frameBuffer + (FRAME_HEIGHT - tile_i * TILE_SIZE - 1) * FRAME_WIDTH + (tile_j * TILE_SIZE);
            for (int i = tile_i * TILE_SIZE; i < (tile_i * TILE_SIZE) + TILE_SIZE; i++)
            {
                float x = tile_x;

                for (int j = tile_j * TILE_SIZE; j < (tile_j * TILE_SIZE) + TILE_SIZE; j++)
                {
                    const glm::vec3 ray_origin = camera.pos + camera.x * x + camera.y * y;

                    core::Ray ray = {ray_origin, ray_direction};

                    bool hit = false;

                    hit = bvh.traversalHybrid(ray, MAX_INTERSECTIONS, cachedRayDirs, useCaching);

                    const float src_alpha = 0.4f;

                    __m128i c = _mm_set1_epi32(0);

                    if (hit)
                    {
                        __m128 cf = _mm_set1_ps(0.0f);

                        if (DebugState::HEATMAP_VIEW)
                        {
                            glm::vec3 heat_map_color = getColorMap(ray.bvh_nodes_visited, 1, 200);
                            cf = _mm_set_ps(1.0f, heat_map_color.z, heat_map_color.y, heat_map_color.x);
                            cf = _mm_min_ps(_mm_mul_ps(cf, _mm_set1_ps(255.0f)), _mm_set1_ps(255.0f));
                            c = _mm_shuffle_epi8(_mm_cvtps_epi32(cf), _mm_set1_epi32(0x0C080400));
                        } else
                        {
                            float dst_alpha = 1.0f;

                            for (int n = 0; n < 3; n++)
                            {
                                const int triangle = ray.triangle[n];

                                // c += COLORS[bvh.getTriangle(ray.triangle[0]).material & 0x07] * (dst_alpha * src_alpha * std::abs(ray.dot[n]));
                                const __m128 abs_dot_x4 = _mm_andnot_ps(_mm_set1_ps(-0.0f),
                                                                        _mm_load1_ps(&ray.dot[n]));
                                const __m128 tri_color = _mm_load_ps(
                                        COLORS[bvh.getTriangle(triangle).material & 0x07].data());
                                const __m128 shaded_color = _mm_mul_ps(tri_color, abs_dot_x4);

                                const float alpha = dst_alpha * src_alpha;

                                cf = _mm_add_ps(cf, _mm_mul_ps(_mm_load1_ps(&alpha), shaded_color));

                                dst_alpha *= (1.0 - src_alpha);
                            }
                            // c = min(255 * cf)
                            cf = _mm_min_ps(_mm_mul_ps(cf, _mm_set1_ps(255.0f)), _mm_set1_ps(255.0f));
                            c = _mm_shuffle_epi8(_mm_cvtps_epi32(cf), _mm_set1_epi32(0x0C080400));
                        }
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
void render_frame_4x4(const core::Camera &camera, const core::BVH &bvh, core::RGBA *const frameBuffer)
{
    auto COLORS = getMaterial();
    const glm::vec3 rd = {1.0f / camera.dir.x, 1.0f / camera.dir.y, 1.0f / camera.dir.z};

    tbb::parallel_for(tbb::blocked_range<int>(0, NX * NY), [&](const tbb::blocked_range<int> &r)
    {
        for (int tile_idx = r.begin(); tile_idx != r.end(); tile_idx++)
        {
            const int tile_i = (tile_idx / NX);
            const int tile_j = (tile_idx % NX);

            for (int bundle_i = 0; bundle_i < TILE_SIZE / BUNDLE_SIZE; bundle_i++)
            {
                const int bundle_py = (tile_i * TILE_SIZE) + (bundle_i * BUNDLE_SIZE);
                const float bundle_y = -(VIEWPORT_HEIGHT / 2.0f) + (bundle_py * DY);

                for (int bundle_j = 0; bundle_j < TILE_SIZE / BUNDLE_SIZE; bundle_j++)
                {
                    const int bundle_px = (tile_j * TILE_SIZE) + (bundle_j * BUNDLE_SIZE);
                    const float bundle_x = -(VIEWPORT_WIDTH / 2.0f) + (bundle_px * DX);

                    core::RGBA *const p =
                            frameBuffer + ((FRAME_HEIGHT - bundle_py - BUNDLE_SIZE) * FRAME_WIDTH) + bundle_px;

                    const glm::vec3 bundle_origin = camera.pos + camera.x * bundle_x + camera.y * bundle_y;

                    core::Ray4x4 rays = {camera, bundle_origin, camera.dir, rd, DX, DY};

                    const bool hit = bvh.traversal4x4(rays, MAX_INTERSECTIONS);

                    __m128 src_alpha = _mm_set1_ps(0.6f);

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
                                const int triangle = rays.triangle[n * 16 + r];

                                // c += COLORS[bvh.getTriangle(ray.triangle[0]).material & 0x07] * (dst_alpha * src_alpha * std::abs(ray.dot[n]));
                                const __m128 abs_dot_x4 = _mm_andnot_ps(_mm_set1_ps(-0.0f),
                                                                        _mm_load1_ps(rays.dot.data() + n * 16 + r));
                                const __m128 tri_color = _mm_load_ps(
                                        COLORS[bvh.getTriangle(triangle).material & 0x07].data());
                                const __m128 shaded_color = _mm_mul_ps(tri_color, abs_dot_x4);

                                const __m128 alpha = _mm_mul_ps(dst_alpha, src_alpha);

                                cf = _mm_add_ps(cf, _mm_mul_ps(alpha, shaded_color));

                                // dst_alpha *= (1.0 - src_alpha);
                                dst_alpha = _mm_mul_ps(dst_alpha, _mm_sub_ps(_mm_set1_ps(1.0f), src_alpha));
                            }
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

// For OBBs
void cacheRayDirs(core::obb::ObbTree &obbTree, std::vector<glm::vec3> &out, const glm::vec3 &rayDir)
{
    std::vector<glm::mat4x4> cachedObbTransformations = obbTree.getTransformationCache();
    for (auto &obbTransformation: cachedObbTransformations)
    {
        out.emplace_back(obbTransformation * glm::vec4(rayDir, 0.0f));
    }
}

void cacheRayDirsNoClustering(core::obb::ObbTree &obbTree, const glm::vec3 &rayDir)
{
    // Loop over all nodes in the tree
    for (auto &node: obbTree.getNodes())
    {
        node.cachedRayDir = glm::vec3((node.obb.invMatrix) * glm::vec4(rayDir, 0.0f));
    }
}

// OBB tree traversal
void render_frameOBB(const core::Camera &camera, core::obb::ObbTree &obbTree, core::RGBA *const frameBuffer,
                     bool useClustering, bool useRayCaching)
{
    ZoneScopedN("Render OBB Tree");
    // BEFORE LOOPING. SHARE READ ONLY
    const glm::vec3 ray_direction = camera.dir;
    std::vector<glm::vec3> cachedRayDirs;

    if (useRayCaching && useClustering)
    {
        cacheRayDirs(obbTree, cachedRayDirs, ray_direction);
    } else if (useRayCaching && !useClustering)
    {
        cacheRayDirsNoClustering(obbTree, ray_direction);
    }

    auto COLORS = getMaterial();
    tbb::parallel_for(tbb::blocked_range<int>(0, NX * NY), [&](const tbb::blocked_range<int> &r)
    {
        for (int tile_idx = r.begin(); tile_idx != r.end(); tile_idx++)
        {
            const int tile_i = (tile_idx / NX);
            const int tile_j = (tile_idx % NX);

            const float tile_y = -(VIEWPORT_HEIGHT / 2.0f) + (tile_i * TILE_SIZE * DY);
            const float tile_x = -(VIEWPORT_WIDTH / 2.0f) + (tile_j * TILE_SIZE * DX);

            float y = tile_y;
            core::RGBA *p = frameBuffer + (FRAME_HEIGHT - tile_i * TILE_SIZE - 1) * FRAME_WIDTH + (tile_j * TILE_SIZE);
            for (int i = tile_i * TILE_SIZE; i < (tile_i * TILE_SIZE) + TILE_SIZE; i++)
            {
                float x = tile_x;

                for (int j = tile_j * TILE_SIZE; j < (tile_j * TILE_SIZE) + TILE_SIZE; j++)
                {
                    const glm::vec3 ray_origin = camera.pos + camera.x * x + camera.y * y;

                    core::Ray ray = {ray_origin, ray_direction};

                    bool hit = obbTree.traversal(ray, MAX_INTERSECTIONS, cachedRayDirs, useRayCaching);

                    const float src_alpha = 0.4f;

                    __m128i c = _mm_set1_epi32(0);

                    if (hit)
                    {
                        __m128 cf = _mm_set1_ps(0.0f);

                        if (DebugState::HEATMAP_VIEW)
                        {
                            glm::vec3 heat_map_color = getColorMap(ray.bvh_nodes_visited, 1, 200);
                            cf = _mm_set_ps(1.0f, heat_map_color.z, heat_map_color.y, heat_map_color.x);
                            cf = _mm_min_ps(_mm_mul_ps(cf, _mm_set1_ps(255.0f)), _mm_set1_ps(255.0f));
                            c = _mm_shuffle_epi8(_mm_cvtps_epi32(cf), _mm_set1_epi32(0x0C080400));
                        } else
                        {
                            float dst_alpha = 1.0f;

                            for (int n = 0; n < 3; n++)
                            {
                                const int triangle = ray.triangle[n];

                                // c += COLORS[bvh.getTriangle(ray.triangle[0]).material & 0x07] * (dst_alpha * src_alpha * std::abs(ray.dot[n]));
                                const __m128 abs_dot_x4 = _mm_andnot_ps(_mm_set1_ps(-0.0f),
                                                                        _mm_load1_ps(&ray.dot[n]));
                                const __m128 tri_color = _mm_load_ps(
                                        COLORS[obbTree.getTriangle(triangle).material & 0x07].data());
                                const __m128 shaded_color = _mm_mul_ps(tri_color, abs_dot_x4);

                                const float alpha = dst_alpha * src_alpha;

                                cf = _mm_add_ps(cf, _mm_mul_ps(_mm_load1_ps(&alpha), shaded_color));

                                dst_alpha *= (1.0 - src_alpha);
                            }
                            // c = min(255 * cf)
                            cf = _mm_min_ps(_mm_mul_ps(cf, _mm_set1_ps(255.0f)), _mm_set1_ps(255.0f));
                            c = _mm_shuffle_epi8(_mm_cvtps_epi32(cf), _mm_set1_epi32(0x0C080400));
                        }
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

void render_frame_4x4OBB(const core::Camera &camera, core::obb::ObbTree &obbTree, core::RGBA *const frameBuffer,
                         bool useClustering, bool useRayCaching)
{
    const glm::vec3 ray_direction = camera.dir;
    std::vector<glm::vec3> cachedRayDirs;

    if (useRayCaching && useClustering)
    {
        cacheRayDirs(obbTree, cachedRayDirs, ray_direction);
    } else if (useRayCaching && !useClustering)
    {
        cacheRayDirsNoClustering(obbTree, ray_direction);
    }

    auto COLORS = getMaterial();
    const glm::vec3 rd = {1.0f / camera.dir.x, 1.0f / camera.dir.y, 1.0f / camera.dir.z};

    tbb::parallel_for(tbb::blocked_range<int>(0, NX * NY), [&](const tbb::blocked_range<int> &r)
    {
        for (int tile_idx = r.begin(); tile_idx != r.end(); tile_idx++)
        {
            const int tile_i = (tile_idx / NX);
            const int tile_j = (tile_idx % NX);

            for (int bundle_i = 0; bundle_i < TILE_SIZE / BUNDLE_SIZE; bundle_i++)
            {
                const int bundle_py = (tile_i * TILE_SIZE) + (bundle_i * BUNDLE_SIZE);
                const float bundle_y = -(VIEWPORT_HEIGHT / 2.0f) + (bundle_py * DY);

                for (int bundle_j = 0; bundle_j < TILE_SIZE / BUNDLE_SIZE; bundle_j++)
                {
                    const int bundle_px = (tile_j * TILE_SIZE) + (bundle_j * BUNDLE_SIZE);
                    const float bundle_x = -(VIEWPORT_WIDTH / 2.0f) + (bundle_px * DX);

                    core::RGBA *const p =
                            frameBuffer + ((FRAME_HEIGHT - bundle_py - BUNDLE_SIZE) * FRAME_WIDTH) + bundle_px;

                    const glm::vec3 bundle_origin = camera.pos + camera.x * bundle_x + camera.y * bundle_y;

                    core::Ray4x4 rays = {camera, bundle_origin, camera.dir, rd, DX, DY};

                    const bool hit = obbTree.traversal4x4(rays, MAX_INTERSECTIONS, cachedRayDirs, useRayCaching);

                    __m128 src_alpha = _mm_set1_ps(0.6f);

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
                                const int triangle = rays.triangle[n * 16 + r];

                                // c += COLORS[bvh.getTriangle(ray.triangle[0]).material & 0x07] * (dst_alpha * src_alpha * std::abs(ray.dot[n]));
                                const __m128 abs_dot_x4 = _mm_andnot_ps(_mm_set1_ps(-0.0f),
                                                                        _mm_load1_ps(rays.dot.data() + n * 16 + r));
                                const __m128 tri_color = _mm_load_ps(
                                        COLORS[obbTree.getTriangle(triangle).material & 0x07].data());
                                const __m128 shaded_color = _mm_mul_ps(tri_color, abs_dot_x4);

                                const __m128 alpha = _mm_mul_ps(dst_alpha, src_alpha);

                                cf = _mm_add_ps(cf, _mm_mul_ps(alpha, shaded_color));

                                // dst_alpha *= (1.0 - src_alpha);
                                dst_alpha = _mm_mul_ps(dst_alpha, _mm_sub_ps(_mm_set1_ps(1.0f), src_alpha));
                            }
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




