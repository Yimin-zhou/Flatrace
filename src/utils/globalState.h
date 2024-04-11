#pragma once

#define LEAF_SIZE 5

#define GEN_OBB_BVH 1
#define OBB_METHOD_1 0

#define VIS_AABB_OBB 0

#define MODEL_NORMALIZE 1
//#define MODEL_SCALE 0.00035f
#define MODEL_SCALE 1.0f
#define MODEL_FLIP 0

struct GlobalState
{
    static bool heatmapView;
    static bool bboxView;
    static bool enableOBB;
};



