#include <gtest/gtest.h>
#include <numeric>

#include "core/types.h"
#include "src/core/bvh/bvh.h"

#include "core/trace.h"
#include "utils/obj.h"
#include "utils/ppm.h"

using namespace core;

namespace test
{

    constexpr int WINDOW_WIDTH = 1280;
    constexpr int WINDOW_HEIGHT = 720;

    constexpr auto VIEWPORT_WIDTH = 2.4f;
    constexpr auto VIEWPORT_HEIGHT = 1.35f;

    constexpr auto m_tileSize = 16;
    constexpr auto m_bundleSize = 4;

    constexpr auto MAX_INTERSECTIONS = 3;

    const std::string TEST_OBJ = "test/input/test/bunny.obj";
    const std::string TEST_OBJ_FOLDER = "test/input/big_obj";

    //  test :loading a simple obj file
    TEST(FlatRace, ObjLoading_1)
    {
        std::vector<Triangle> test_model = utils::Obj::read(TEST_OBJ, true);
    }

    // test : loading multiple obj files
    TEST(FlatRace, ObjLoading_2)
    {
        std::vector<std::vector<Triangle>> models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER, false);
    }

    //  test :simple test the construction of the aabb BVH
    TEST(FlatRace, Construction_1)
    {
        const std::vector<std::pair<int, glm::vec3>> POINTS = {
                {0, {-20.0f, -20.0f, 0.0f}},
                {1, {11.0f,  21.0f,  0.0f}},
                {2, {-20.0f, -10.0f, 0.0f}},
                {3, {21.0f,  11.0f,  0.0f}},
                {4, {12.0f,  -12.0f, 42.0f}},
                {5, {-10.0f, -20.0f, 0.0f}},
        };

        std::vector<Triangle> triangles(POINTS.size());
        std::transform(POINTS.begin(), POINTS.end(), triangles.begin(),
                       [](const auto &v) -> Triangle { return {v.first, v.second, v.second, v.second, 0}; });

        AABBTree aabbTree(triangles);
    }

    //  test :simple test the construction of the obb BVH
    TEST(FlatRace, Construction_2)
    {
        const std::vector<std::pair<int, glm::vec3>> POINTS = {
                {0, {-20.0f, -20.0f, 0.0f}},
                {1, {11.0f,  21.0f,  0.0f}},
                {2, {-20.0f, -10.0f, 0.0f}},
                {3, {21.0f,  11.0f,  0.0f}},
                {4, {12.0f,  -12.0f, 42.0f}},
                {5, {-10.0f, -20.0f, 0.0f}},
        };

        std::vector<Triangle> triangles(POINTS.size());
        std::transform(POINTS.begin(), POINTS.end(), triangles.begin(),
                       [](const auto &v) -> Triangle { return {v.first, v.second, v.second, v.second, 0}; });

        std::vector<std::vector<Triangle>> models = {triangles};
        ObbTree obbTree(models);
    }

    //  test :write results to a file
    TEST(FlatRace, Render_1)
    {
        std::vector<std::vector<Triangle>> models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER, false);
        Tracer tracer_aabb_1 = Tracer(models, WINDOW_WIDTH, WINDOW_HEIGHT, MAX_INTERSECTIONS,
                                      VIEWPORT_WIDTH, VIEWPORT_HEIGHT, m_tileSize, m_bundleSize, false);
        Tracer tracer_aabb_2 = Tracer(models, WINDOW_WIDTH, WINDOW_HEIGHT, MAX_INTERSECTIONS,
                                      VIEWPORT_WIDTH, VIEWPORT_HEIGHT, m_tileSize, m_bundleSize, false);

        const float cx = std::cos(2.0f) * 2.0f;
        const float cz = std::sin(2.0f) * 2.0f;
        glm::vec3 position = {cx, 1.0f, cz};
        glm::vec3 target = {-cx, -1.0f, -cz};
        glm::vec3 up = {0.0f, 1.0f, 0.0f};
        float zoom = 5.0f;

        Camera camera = {position, target, up, zoom};

        // AABB
        tracer_aabb_1.render(camera, false);
        // OBB
        tracer_aabb_2.render(camera, true);

        // save the results
//        utils::Ppm::write("test/output/writeTest/aabb.ppm", tracer_aabb_1.getFrame());
//        utils::Ppm::write("test/output/writeTest/obb.ppm", tracer_aabb_2.getFrame());
    }

    bool compareFrames(const core::Frame &frame1, const core::Frame &frame2)
    {
        if (frame1.width != frame2.width || frame1.height != frame2.height)
        {
            return false; // Dimensions mismatch
        }

        int count = 0;

        for (int i = 0; i < frame1.width * frame1.height; ++i)
        {
            // Compare pixel values, ignoring the small differences
            if (std::abs(frame1.pixels[i].r - frame2.pixels[i].r) > 1 ||
                std::abs(frame1.pixels[i].g - frame2.pixels[i].g) > 1 ||
                std::abs(frame1.pixels[i].b - frame2.pixels[i].b) > 1 ||
                std::abs(frame1.pixels[i].a - frame2.pixels[i].a) > 1)
            {
                count++;
            }
        }
        if (count > 5) // Allow 5 pixel to be different
        {
            return false;
        } else
        {
            return true;
        }
    }

    //  compare the results of aabb and obb (AABB tree)
    TEST(FlatRace, Render_2)
    {
        std::vector<std::vector<Triangle>> models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER, false);
        Tracer tracer_aabb_1 = Tracer(models, WINDOW_WIDTH, WINDOW_HEIGHT, MAX_INTERSECTIONS,
                                    VIEWPORT_WIDTH, VIEWPORT_HEIGHT, m_tileSize, m_bundleSize, false);
        Tracer tracer_aabb_2 = Tracer(models, WINDOW_WIDTH, WINDOW_HEIGHT, MAX_INTERSECTIONS,
                                      VIEWPORT_WIDTH, VIEWPORT_HEIGHT, m_tileSize, m_bundleSize, false);

        const float cx = std::cos(2.0f) * 2.0f;
        const float cz = std::sin(2.0f) * 2.0f;
        glm::vec3 position = {cx, 1.0f, cz};
        glm::vec3 target = {-cx, -1.0f, -cz};
        glm::vec3 up = {0.0f, 1.0f, 0.0f};
        float zoom = 5.0f;

        Camera camera = {position, target, up, zoom};

        // AABB
        tracer_aabb_1.render(camera, false);
        // OBB
        tracer_aabb_2.render(camera, true);

        // compare the results
        const core::Frame &frame_aabb = tracer_aabb_1.getFrame();
        const core::Frame &frame_obb = tracer_aabb_2.getFrame();

        ASSERT_TRUE(compareFrames(frame_aabb, frame_obb));
    }

    //  test :compare the results of aabb and obb (AABB tree vs OBB tree)
    TEST(FlatRace, Render_3)
    {
        std::vector<std::vector<Triangle>> models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER, false);
        Tracer tracer_aabb = Tracer(models, WINDOW_WIDTH, WINDOW_HEIGHT, MAX_INTERSECTIONS,
                                    VIEWPORT_WIDTH, VIEWPORT_HEIGHT, m_tileSize, m_bundleSize, false);

        Tracer tracer_obb = Tracer(models, WINDOW_WIDTH, WINDOW_HEIGHT, MAX_INTERSECTIONS,
                                   VIEWPORT_WIDTH, VIEWPORT_HEIGHT, m_tileSize, m_bundleSize, true);

        const float cx = std::cos(2.0f) * 2.0f;
        const float cz = std::sin(2.0f) * 2.0f;
        glm::vec3 position = {cx, 1.0f, cz};
        glm::vec3 target = {-cx, -1.0f, -cz};
        glm::vec3 up = {0.0f, 1.0f, 0.0f};
        float zoom = 5.0f;

        Camera camera = {position, target, up, zoom};

        tracer_aabb.render(camera, false);
        tracer_obb.render(camera, false);

        // compare the results
        const core::Frame &frame_aabb = tracer_aabb.getFrame();
        const core::Frame &frame_obb = tracer_obb.getFrame();

        ASSERT_TRUE(compareFrames(frame_aabb, frame_obb));
    }

    // test : compare the results of aabb and obb from 10 angles (AABB tree vs OBB tree)
    TEST(FlatRace, Render_4)
    {
        std::vector<std::vector<Triangle>> models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER, false);
        Tracer tracer_aabb = Tracer(models, WINDOW_WIDTH, WINDOW_HEIGHT, MAX_INTERSECTIONS,
                                    VIEWPORT_WIDTH, VIEWPORT_HEIGHT, m_tileSize, m_bundleSize, false);

        Tracer tracer_obb = Tracer(models, WINDOW_WIDTH, WINDOW_HEIGHT, MAX_INTERSECTIONS,
                                   VIEWPORT_WIDTH, VIEWPORT_HEIGHT, m_tileSize, m_bundleSize, true);

        glm::vec3 position;
        glm::vec3 target = {0.0f, 0.0f, 0.0f};
        glm::vec3 up = {0.0f, 1.0f, 0.0f};
        float zoom = 5.0f;

        // rotate camera around the object from different positions
        for (int i = 0; i < 6; ++i)
        {
            const float cx = std::cos(i) * 2.0f;
            const float cz = std::sin(i) * 2.0f;
            Camera camera = {{cx, 1.0f, cz}, target, up, zoom};
            tracer_aabb.render(camera, false);
            tracer_obb.render(camera, false);
            // compare the results
            const core::Frame &frame_aabb = tracer_aabb.getFrame();
            const core::Frame &frame_obb = tracer_obb.getFrame();
            ASSERT_TRUE(compareFrames(frame_aabb, frame_obb));
        }

        for (int i = 0; i < 6; ++i)
        {
            const float cx = std::cos(i) * 2.0f;
            const float cz = std::sin(i) * 2.0f;
            Camera camera = {{cx, -1.0f, cz}, target, up, zoom};
            tracer_aabb.render(camera, false);
            tracer_obb.render(camera, false);
            // compare the results
            const core::Frame &frame_aabb = tracer_aabb.getFrame();
            const core::Frame &frame_obb = tracer_obb.getFrame();
            ASSERT_TRUE(compareFrames(frame_aabb, frame_obb));
        }
    }

    // test 7: profile the performance of the BVH construction

    // test 8: profile the performance of the BVH traversal, from different camera positions
    TEST(FlatRace, Perfomance_2)
    {
        std::vector<std::vector<Triangle>> models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER, false);
        Tracer tracer_aabb = Tracer(models, WINDOW_WIDTH, WINDOW_HEIGHT, MAX_INTERSECTIONS,
                                    VIEWPORT_WIDTH, VIEWPORT_HEIGHT, m_tileSize, m_bundleSize, false);

        Tracer tracer_obb = Tracer(models, WINDOW_WIDTH, WINDOW_HEIGHT, MAX_INTERSECTIONS,
                                   VIEWPORT_WIDTH, VIEWPORT_HEIGHT, m_tileSize, m_bundleSize, true);

        glm::vec3 position;
        glm::vec3 target = {0.0f, 0.0f, 0.0f};
        glm::vec3 up = {0.0f, 1.0f, 0.0f};
        float zoom = 5.0f;

        // rotate camera around the object from different positions, record the time
        // and calculate the average time of 2 methods
        std::vector<float> aabb_times;
        std::vector<float> obb_times;
        for (int i = 0; i < 6; ++i)
        {
            const float cx = std::cos(i) * 2.0f;
            const float cz = std::sin(i) * 2.0f;
            Camera camera = {{cx, 1.0f, cz}, target, up, zoom};

            // record the time
            auto start = std::chrono::high_resolution_clock::now();
            tracer_aabb.render(camera, false);
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<float> diff = end - start;
            aabb_times.push_back(diff.count());

            start = std::chrono::high_resolution_clock::now();
            tracer_obb.render(camera, false);
            end = std::chrono::high_resolution_clock::now();
            diff = end - start;
            obb_times.push_back(diff.count());
        }

        // calculate the average time
        float aabb_avg_time = std::accumulate(aabb_times.begin(), aabb_times.end(), 0.0f) / aabb_times.size();
        float obb_avg_time = std::accumulate(obb_times.begin(), obb_times.end(), 0.0f) / obb_times.size();

        std::cout << "Average time of AABB: " << aabb_avg_time * 1000 << " ms" << std::endl;
        std::cout << "Average time of OBB: " << obb_avg_time * 1000 << " ms" << std::endl;
    }

}