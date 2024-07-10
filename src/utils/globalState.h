#pragma once

#define LEAF_SIZE 1

#define ENABLE_AABB_WITH_OBB 0
#define ENABLE_CACHING 0

#define ENABLE_OBB_BVH 1
#define ENABLE_CLUSTERING 0
#define NUM_CLUSTERS 10

#define MODEL_NORMALIZE 1
//#define MODEL_SCALE 0.00035f
#define MODEL_SCALE 1.0f
#define MODEL_FLIP 0

struct GlobalState
{
    static bool heatmapView;
    static bool bboxView;
};
