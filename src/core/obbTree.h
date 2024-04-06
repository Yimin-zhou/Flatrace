#pragma once

#include "types.h"
#include "intersect.h"
#include "dito/dito.h"
#include "imgui/imgui.h"
#include "bvh.h"

#include <vector>
#include <optional>

namespace core {

    class ObbTree {
    public:
        ObbTree(const std::vector<std::vector<Triangle>>& objects);
        bool intersect(Ray &ray, const int maxIntersections) const;

    private:
        std::vector<Triangle> _triangles;
        std::vector<Node> _nodes;
        unsigned int _nodeCount;
        Node *_root;

        BoundingBox _unitAABB;

        void init(const std::vector<Triangle>& triangles, unsigned int nodeIndex);
        void computeOBB(const std::vector<Triangle>& object, Node* node);
    };

}

