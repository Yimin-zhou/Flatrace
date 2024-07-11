#pragma once

#define LEAF_SIZE 1

// Optimization
#define ENABLE_CACHING 0

// AABB Tree
#define ENABLE_AABB_WITH_OBB 0

// OBB Tree
#define ENABLE_OBB_BVH 1
#define ENABLE_CLUSTERING 0
#define NUM_CLUSTERS 10
#define ENABLE_OBB_SAH 1

// Hybrid Tree
#define ENABLE_HYBRID_BVH 0

#define MODEL_NORMALIZE 1
//#define MODEL_SCALE 0.00035f
#define MODEL_SCALE 1.0f
#define MODEL_FLIP 0

struct GlobalState
{
    static bool heatmapView;
    static bool bboxView;
};
