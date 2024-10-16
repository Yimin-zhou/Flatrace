#pragma once

constexpr int WINDOW_WIDTH = 1280;
constexpr int WINDOW_HEIGHT = 720;

constexpr int FRAME_WIDTH = 1280;
constexpr int FRAME_HEIGHT = 720;

constexpr auto VIEWPORT_WIDTH = 2.4f;
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

struct TracerState
{
    static int LEAF_SIZE;
//    static bool ENABLE_CACHING;
    static int HEATMAP_SIZE;

    // AABB Tree
    static bool ENABLE_AABB_WITH_OBB;
    static bool ENABLE_AABB_SIMD;

    // OBB Tree
    static bool ENABLE_OBB_BVH;
    static bool ENABLE_CLUSTERING;
    static bool ENABLE_OBB_SAH;
    static bool ENABLE_OBB_MEDIAN;
    static int NUM_CLUSTERS;
    static int NUM_BINS;
    static bool ENABLE_OBB_SIMD;

    // Hybrid Tree
    static bool ENABLE_HYBRID_BVH;
    static bool MODEL_NORMALIZE;
    static float MODEL_SCALE;
    static bool MODEL_FLIP;
};

struct DebugState
{
    static bool HEATMAP_VIEW;
    static bool BBOX_VIEW;
};
