#include "trace.h"
#include "src/utils/globalState.h"

#include <chrono>
#include <string>
#include <vector>


core::Tracer::Tracer(const std::vector<std::vector<Triangle>> &meshes, int width,
                     int height, int maxIterations, float viewWidth,
                     float viewHeight, int tileSize, int bundleSize, bool genObbBvh) :
        m_meshes(meshes),
        m_width(width),
        m_height(height),
        m_viewportWidth(viewWidth),
        m_viewportHeight(viewHeight),
        m_maxIterations(maxIterations),
        m_tileSize(tileSize),
        m_bundleSize(bundleSize),
        m_frame(width, height)
{
    m_dx = m_viewportWidth / m_width;
    m_dy = m_viewportHeight / m_height;
    m_nx = m_width / m_tileSize;
    m_ny = m_height / m_tileSize;

    // add all triangles from all objects
    std::vector<Triangle> model;
    for (int i = 0; i < m_meshes.size(); i++)
    {
        model.insert(model.end(), m_meshes[i].begin(), m_meshes[i].end());
    }

#if MODEL_FLIP
    std::transform(model.begin(), model.end(), model.begin(), [](const Triangle &t) -> Triangle
    {
        const glm::vec3 &v0f = {t.vertices[0].x, t.vertices[0].z, t.vertices[0].y};
        const glm::vec3 &v1f = {t.vertices[1].x, t.vertices[1].z, t.vertices[1].y};
        const glm::vec3 &v2f = {t.vertices[2].x, t.vertices[2].z, t.vertices[2].y};

        return {t.id, v0f, v2f, v1f, t.material};
    });
#endif

    if (genObbBvh)
    {
        // init to obb tree
        m_bvh = std::make_shared<core::ObbTree>(m_meshes);
    }
    else
    {
#if ENABLE_OBB_BVH
        m_bvh = std::make_shared<core::ObbTree>(m_meshes);
#else
        m_bvh = std::make_shared<core::AABBTree>(model);
#endif
    }

    m_visualization = debug::Visualization(m_bvh);
    std::cout << "\nGenerating Triangles for BBox..." << std::endl;
    m_bboxBVH = std::make_shared<core::AABBTree>(m_visualization.getTriangles());
}

void core::Tracer::resize(int width, int height)
{
    int targetHeight = width * 9 / 16;
    if (targetHeight > height)
    {
        width = height * 16 / 9;
    }
    else
    {
        height = targetHeight;
    }

    m_width = width;
    m_height = height;
    m_frame = Frame(m_width, m_height);
    m_dx = m_viewportWidth / static_cast<float>(m_width);
    m_dy = m_viewportHeight / static_cast<float>(m_height);

    m_nx = m_width / m_tileSize;
    m_ny = m_height / m_tileSize;

}

void core::Tracer::render(const core::Camera &camera, bool traverseObb)
{
    if (GlobalState::bboxView)
    {
        renderBboxFrame(camera, traverseObb);
    }
    else
    {
        renderFrame(camera, traverseObb);
    }
}

// Reference implementation that traces 1 ray at a time (no SIMD)
void core::Tracer::renderFrame(const core::Camera &camera, bool traverseObbInAabb)
{
    auto COLORS = getMaterial();

    tbb::parallel_for(tbb::blocked_range<int>(0, m_nx * m_ny), [&](const tbb::blocked_range<int> &r)
    {
        for (int tile_idx = r.begin(); tile_idx != r.end(); tile_idx++)
        {
            const int tile_i = (tile_idx / m_nx);
            const int tile_j = (tile_idx % (int)m_nx);

            const float tile_y = -(m_viewportHeight / 2.0f) + (tile_i * m_tileSize * m_dy);
            const float tile_x = -(m_viewportWidth / 2.0f) + (tile_j * m_tileSize * m_dx);

            float y = tile_y;
            core::RGBA *p = m_frame.pixels.get() + (m_height - tile_i * m_tileSize - 1) * m_width + (tile_j * m_tileSize);
            for (int i = tile_i * m_tileSize; i < (tile_i * m_tileSize) + m_tileSize; i++)
            {
                float x = tile_x;

                for (int j = tile_j * m_tileSize; j < (tile_j * m_tileSize) + m_tileSize; j++)
                {
                    auto start = std::chrono::high_resolution_clock::now();

                    const glm::vec3 ray_origin = camera.pos + camera.x * x + camera.y * y;
                    const glm::vec3 ray_direction = camera.dir;

                    core::Ray ray = {ray_origin, ray_direction};

                    bool hit = false;

                    if (traverseObbInAabb || (GlobalState::enableOBB && !ENABLE_OBB_BVH))
                    {
                        hit = m_bvh->traversalOBB(ray, m_maxIterations);
                    } else
                    {
                        hit = m_bvh->traversal(ray, m_maxIterations);
                    }

                    auto end = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<float, std::milli> processingTime = end - start;

//                    if (_rayCount % _sampleRate == 0) {
//                    if (_rayCount < NX*NY - 1)
//                    rayProcessingTimes[_rayCount] = processingTime.count();
//                    }
                    _rayCount++;

                    const float src_alpha = 0.4f;

                    __m128i c = _mm_set1_epi32(0);

                    if (hit)
                    {
                        __m128 cf = _mm_set1_ps(0.0f);

                        if (GlobalState::heatmapView)
                        {
                            glm::vec3 heat_map_color = getColorMap(
                                    ray.bvh_nodes_visited, 1, 300);
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
                                        COLORS[m_bvh->getTriangle(triangle).material & 0x07].data());
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

                    x += m_dx;
                    p += 1;
                }

                y += m_dy;
                p -= (m_width + m_tileSize);
            }
        }
    });
}

// 8-way SIMD implementation that traces 4x4 'ray bundles'
void core::Tracer::renderFrame4X4(const std::unique_ptr<BVH> bvh, const core::Camera &camera)
{
    auto COLORS = getMaterial();
    const glm::vec3 rd = {1.0f / camera.dir.x, 1.0f / camera.dir.y, 1.0f / camera.dir.z};

    tbb::parallel_for(tbb::blocked_range<int>(0, m_nx * m_ny), [&](const tbb::blocked_range<int> &r)
    {
        for (int tile_idx = r.begin(); tile_idx != r.end(); tile_idx++)
        {
            const int tile_i = (tile_idx / m_nx);
            const int tile_j = (tile_idx % (int)m_nx);

            for (int bundle_i = 0; bundle_i < m_tileSize / m_bundleSize; bundle_i++)
            {
                const int bundle_py = (tile_i * m_tileSize) + (bundle_i * m_bundleSize);
                const float bundle_y = -(m_viewportHeight / 2.0f) + (bundle_py * m_dy);

                for (int bundle_j = 0; bundle_j < m_tileSize / m_bundleSize; bundle_j++)
                {
                    const int bundle_px = (tile_j * m_tileSize) + (bundle_j * m_bundleSize);
                    const float bundle_x = -(m_viewportWidth / 2.0f) + (bundle_px * m_dx);

                    core::RGBA *const p =
                            m_frame.pixels.get() + ((m_height - bundle_py - m_bundleSize) * m_width) + bundle_px;

                    const glm::vec3 bundle_origin = camera.pos + camera.x * bundle_x + camera.y * bundle_y;

                    core::Ray4x4 rays = {camera, bundle_origin, camera.dir, rd, m_dx, m_dy};

                    const bool hit = bvh->traversal4x4(rays, m_maxIterations);

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
                                        COLORS[bvh->getTriangle(triangle).material & 0x07].data());
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

                        _mm_storeu_si32(p + ((m_bundleSize - ray_i - 1) * m_width) + ray_j, c);
                    }
                }
            }
        }
    });
}

// Arbitrary color palette for materials
std::array<std::array<float, 4>, 8> core::Tracer::getMaterial()
{
    std::array<std::array<float, 4>, 8> COLORS;
    if (GlobalState::bboxView)
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
glm::vec3 core::Tracer::getColorMap(int value, int minVal, int maxVal)
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

void core::Tracer::renderBboxFrame(const core::Camera &camera, bool traverseObbInAabb)
{
    auto COLORS = getMaterial();

    tbb::parallel_for(tbb::blocked_range<int>(0, m_nx * m_ny), [&](const tbb::blocked_range<int> &r)
    {
        for (int tile_idx = r.begin(); tile_idx != r.end(); tile_idx++)
        {
            const int tile_i = (tile_idx / m_nx);
            const int tile_j = (tile_idx % (int)m_nx);

            const float tile_y = -(m_viewportHeight / 2.0f) + (tile_i * m_tileSize * m_dy);
            const float tile_x = -(m_viewportWidth / 2.0f) + (tile_j * m_tileSize * m_dx);

            float y = tile_y;
            core::RGBA *p = m_frame.pixels.get() + (m_height - tile_i * m_tileSize - 1) * m_width + (tile_j * m_tileSize);
            for (int i = tile_i * m_tileSize; i < (tile_i * m_tileSize) + m_tileSize; i++)
            {
                float x = tile_x;

                for (int j = tile_j * m_tileSize; j < (tile_j * m_tileSize) + m_tileSize; j++)
                {
                    auto start = std::chrono::high_resolution_clock::now();

                    const glm::vec3 ray_origin = camera.pos + camera.x * x + camera.y * y;
                    const glm::vec3 ray_direction = camera.dir;

                    core::Ray ray = {ray_origin, ray_direction};

                    bool hit = false;

                    hit = m_bboxBVH->traversal(ray, m_maxIterations);

                    auto end = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<float, std::milli> processingTime = end - start;

                    const float src_alpha = 0.4f;

                    __m128i c = _mm_set1_epi32(0);

                    if (hit)
                    {
                        __m128 cf = _mm_set1_ps(0.0f);

                        if (GlobalState::heatmapView)
                        {
                            glm::vec3 heat_map_color = getColorMap(
                                    ray.bvh_nodes_visited, 1, 300);
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
                                        COLORS[m_bboxBVH->getTriangle(triangle).material & 0x07].data());
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

                    x += m_dx;
                    p += 1;
                }

                y += m_dy;
                p -= (m_width + m_tileSize);
            }
        }
    });
}





