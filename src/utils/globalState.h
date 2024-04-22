#pragma once

#define LEAF_SIZE 5

#define ENABLE_OBB_BVH 0 // DISABLE WHEN TESTING
#define OBB_METHOD_1 0

#define VIS_AABB_OBB 0

#define MODEL_NORMALIZE 0
#define MODEL_SCALE 0.00035f
//#define MODEL_SCALE 1.0f
#define MODEL_FLIP 0

struct GlobalState
{
    static bool heatmapView;
    static bool bboxView;
    static bool enableOBB;
};



