#pragma once

struct TracerState
{
    static int LEAF_SIZE;
    static bool ENABLE_CACHING;

    // AABB Tree
    static bool ENABLE_AABB_WITH_OBB;

    // OBB Tree
    static bool ENABLE_OBB_BVH;
    static bool ENABLE_CLUSTERING;
    static int NUM_CLUSTERS;
    static bool ENABLE_OBB_SAH;
    static int NUM_BINS;

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
