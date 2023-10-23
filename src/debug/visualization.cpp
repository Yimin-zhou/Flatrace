//TODO Not fully implemented yet

#include "visualization.h"

namespace debug {
    // Box debug object
    Box::Box(const core::BoundingBox &bbox)
    {
        // Lower bounds
        float xmin = bbox.min.x;
        float ymin = bbox.min.y;
        float zmin = bbox.min.z;

        // Upper bounds
        float xmax = bbox.max.x;
        float ymax = bbox.max.y;
        float zmax = bbox.max.z;

        // Define the 8 vertices of the box
        std::vector<core::Vec3> vertices = {
                core::Vec3(xmin, ymin, zmin),
                core::Vec3(xmax, ymin, zmin),
                core::Vec3(xmax, ymax, zmin),
                core::Vec3(xmin, ymax, zmin),
                core::Vec3(xmin, ymin, zmax),
                core::Vec3(xmax, ymin, zmax),
                core::Vec3(xmax, ymax, zmax),
                core::Vec3(xmin, ymax, zmax)
        };

        // Define the 12 triangles of the box
        this->lines = {
                Line(vertices[0], vertices[1]), // bottom face
                Line(vertices[1], vertices[2]),
                Line(vertices[2], vertices[3]),
                Line(vertices[3], vertices[0]),
                Line(vertices[4], vertices[5]), // top face
                Line(vertices[5], vertices[6]),
                Line(vertices[6], vertices[7]),
                Line(vertices[7], vertices[4]),
                Line(vertices[0], vertices[4]), // front face
                Line(vertices[1], vertices[5]),
                Line(vertices[2], vertices[6]), // back face
                Line(vertices[3], vertices[7])
        };
    }

    // Visualization debug object
//    std::vector<core::Triangle> Visualization::visualize(const core::BVH &bvh)
//    {
//        // construct a box object for each bounding box
//    }
}
