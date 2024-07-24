#include "globalState.h"

// Optimization
int TracerState::LEAF_SIZE = 15;
bool TracerState::ENABLE_CACHING = false;

// AABB Tree
bool TracerState::ENABLE_AABB_WITH_OBB = false;

// OBB Tree
bool TracerState::ENABLE_OBB_BVH = true;
bool TracerState::ENABLE_CLUSTERING = false;
int TracerState::NUM_CLUSTERS = 10;
bool TracerState::ENABLE_OBB_SAH = false;
int TracerState::NUM_BINS = 16;

// Hybrid Tree
bool TracerState::ENABLE_HYBRID_BVH = false;

bool TracerState::MODEL_NORMALIZE = false;
float TracerState::MODEL_SCALE = 0.00035f;
// float GlobalState::MODEL_SCALE = 1.0f;
bool TracerState::MODEL_FLIP = false;

bool DebugState::HEATMAP_VIEW = false;
bool DebugState::BBOX_VIEW = false;
