#pragma once

#define LEAF_SIZE 3

#define ENABLE_OBB_BVH 1 // DISABLE WHEN TESTING
#define ENABLE_OBB_TRACING 0 // FOR TESTING, OBB in AABB Tree, it should be 0 not in testing

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
