#include <gtest/gtest.h>
#include <numeric>

#include "src/core/types.h"
#include "src/core/bvh.h"
#include "src/core/obbTree.h"

#include "src/core/trace.h"
#include "src/utils/obj.h"
#include "src/utils/ppm.h"

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

    const std::string TEST_OBJ = "test/input/house_interior.obj";
    const std::string TEST_OBJ_FOLDER_Bunny = "test/input/test";
    const std::string TEST_OBJ_FOLDER_Semi = "test/input/big_obj";

    //  test :loading a simple obj file
    TEST(FlatRace, ObjLoading_1)
    {
        std::vector<Triangle> test_model = utils::Obj::read(TEST_OBJ, true);
    }

    // test : loading multiple obj files
    TEST(FlatRace, ObjLoading_2)
    {
        std::vector<std::vector<Triangle>> models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Semi, false);
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

        BVH bvh(triangles);
        if (bvh.failed())
        {
            std::cerr << "BVH construction failed" << std::endl;
        }

        Frame frame(FRAME_WIDTH, FRAME_HEIGHT);

        // camera
        const float cx = std::cos(0.0f) * 2.0f;
        const float cz = std::sin(0.0f) * 2.0f;
        Camera camera = {{cx, 1.0f, cz},
                         {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};
        int maxDepth = bvh.getMaxDepth();
        render_frame(camera, bvh, frame.pixels.get(), maxDepth);
    }

    //  test : aabb BVH (Bunny)
    TEST(FlatRace, Render_1)
    {
        std::vector<std::vector<Triangle>> models;
        models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Bunny, true);
        std::vector<Triangle> triangles;
        for (const auto &model : models)
        {
            triangles.insert(triangles.end(), model.begin(), model.end());
        }

        BVH bvh(triangles);
        if (bvh.failed())
        {
            std::cerr << "BVH construction failed" << std::endl;
        }

        Frame frame(FRAME_WIDTH, FRAME_HEIGHT);

        // camera
        const float cx = std::cos(0.0f) * 2.0f;
        const float cz = std::sin(0.0f) * 2.0f;
        Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};
        int maxDepth = bvh.getMaxDepth();

        auto start = std::chrono::high_resolution_clock::now();
        render_frame(camera, bvh, frame.pixels.get(), false);
        auto end = std::chrono::high_resolution_clock::now();
        auto time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "Render Bunny Time taken: " << time << " ms" << std::endl;
        ASSERT_LE(time, 15);
    }

    //  test :aabb BVH (obb tracing) (Bunny)
    TEST(FlatRace, Render_2)
    {
        std::vector<std::vector<Triangle>> models;
        models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Bunny, true);
        std::vector<Triangle> triangles;
        for (const auto &model : models)
        {
            triangles.insert(triangles.end(), model.begin(), model.end());
        }

        BVH bvh(triangles);
        if (bvh.failed())
        {
            std::cerr << "BVH construction failed" << std::endl;
        }

        Frame frame(FRAME_WIDTH, FRAME_HEIGHT);

        // camera
        const float cx = std::cos(0.0f) * 2.0f;
        const float cz = std::sin(0.0f) * 2.0f;
        Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};
        int maxDepth = bvh.getMaxDepth();

        auto start = std::chrono::high_resolution_clock::now();
        render_frame(camera, bvh, frame.pixels.get(), true);
        auto end = std::chrono::high_resolution_clock::now();
        auto time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "Render Bunny OBB in AABB Time taken: " << time << " ms" << std::endl;
    }

    //  test : obb BVH (Bunny)
    TEST(FlatRace, Render_3)
    {
        std::vector<std::vector<Triangle>> models;
        models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Bunny, true);
        std::vector<Triangle> triangles;
        for (const auto &model : models)
        {
            triangles.insert(triangles.end(), model.begin(), model.end());
        }

        core::obb::ObbTree obbTree(triangles);
        if (obbTree.failed())
        {
            std::cerr << "obbTree construction failed" << std::endl;
        }

        Frame frame(FRAME_WIDTH, FRAME_HEIGHT);

        // camera
        const float cx = std::cos(0.0f) * 2.0f;
        const float cz = std::sin(0.0f) * 2.0f;
        Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};

        auto start = std::chrono::high_resolution_clock::now();
        render_frameOBB(camera, obbTree, frame.pixels.get());
        auto end = std::chrono::high_resolution_clock::now();
        auto time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "Render Bunny OBB in OBB Tree Time taken: " << time << " ms" << std::endl;
    }

    //  test :compare obb tree, aabb, aabb with obb (bunny)
    TEST(FlatRace, Render_4)
    {
        std::vector<std::vector<Triangle>> models;
        models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Bunny, true);
        std::vector<Triangle> triangles;
        for (const auto &model : models)
        {
            triangles.insert(triangles.end(), model.begin(), model.end());
        }

        core::obb::ObbTree obbTree(triangles);
        if (obbTree.failed())
        {
            std::cerr << "obbTree construction failed" << std::endl;
        }
        BVH bvh(triangles);

        Frame frame(FRAME_WIDTH, FRAME_HEIGHT);

        // camera
        const float cx = std::cos(0.0f) * 2.0f;
        const float cz = std::sin(0.0f) * 2.0f;
        Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};

        auto start = std::chrono::high_resolution_clock::now();
        render_frameOBB(camera, obbTree, frame.pixels.get());
        auto end = std::chrono::high_resolution_clock::now();
        auto time_obb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        start = std::chrono::high_resolution_clock::now();
        render_frame(camera, bvh, frame.pixels.get(), false);
        end = std::chrono::high_resolution_clock::now();
        auto time_aabb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        start = std::chrono::high_resolution_clock::now();
        render_frame(camera, bvh, frame.pixels.get(), true);
        end = std::chrono::high_resolution_clock::now();
        auto time_aabb_obb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        std::cout << "Render Bunny AABB Time taken: " << time_aabb << " ms" << std::endl;
        std::cout << "Render Bunny OBB in AABB Time taken: " << time_aabb_obb << " ms" << std::endl;
        std::cout << "Render Bunny OBB in OBB Tree Time taken: " << time_obb << " ms" << std::endl;
    }

    //  test :compare obb tree, aabb, aabb with obb (semiconductor)
    TEST(FlatRace, Render_5)
    {
        std::vector<std::vector<Triangle>> models;
        models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Semi, false);
        std::vector<Triangle> triangles;
        for (const auto &model : models)
        {
            triangles.insert(triangles.end(), model.begin(), model.end());
        }

        core::obb::ObbTree obbTree(triangles);
        if (obbTree.failed())
        {
            std::cerr << "obbTree construction failed" << std::endl;
        }
        BVH bvh(triangles);

        Frame frame(FRAME_WIDTH, FRAME_HEIGHT);

        // camera
        const float cx = std::cos(0.0f) * 2.0f;
        const float cz = std::sin(0.0f) * 2.0f;
        Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};

        auto start = std::chrono::high_resolution_clock::now();
        render_frameOBB(camera, obbTree, frame.pixels.get());
        auto end = std::chrono::high_resolution_clock::now();
        auto time_obb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        start = std::chrono::high_resolution_clock::now();
        render_frame(camera, bvh, frame.pixels.get(), false);
        end = std::chrono::high_resolution_clock::now();
        auto time_aabb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        start = std::chrono::high_resolution_clock::now();
        render_frame(camera, bvh, frame.pixels.get(), true);
        end = std::chrono::high_resolution_clock::now();
        auto time_aabb_obb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        std::cout << "Render Semiconductor AABB Time taken: " << time_aabb << " ms" << std::endl;
        std::cout << "Render Semiconductor OBB in AABB Time taken: " << time_aabb_obb << " ms" << std::endl;
        std::cout << "Render Semiconductor OBB in OBB Tree Time taken: " << time_obb << " ms" << std::endl;
    }

    //  test bvh
    TEST(FlatRace, BVH_1)
    {
        std::vector<std::vector<Triangle>> models;
        models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Semi, false);
        std::vector<Triangle> triangles;
        for (const auto &model : models)
        {
            triangles.insert(triangles.end(), model.begin(), model.end());
        }

        std::cout << "Generating OBB Tree ... " << std::endl;

        auto start = std::chrono::high_resolution_clock::now();
        core::obb::ObbTree obbTree(triangles);
        auto end = std::chrono::high_resolution_clock::now();
        auto time_obb_construction = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "OBB Tree Construction Time taken: " << time_obb_construction << " ms" << std::endl;
        if (obbTree.failed())
        {
            std::cerr << "obbTree construction failed" << std::endl;
        }

        // calculate standard deviation of the leaf depths
        std::vector<int> leafDepths = obbTree.getLeafDepths();
        float mean = std::accumulate(leafDepths.begin(), leafDepths.end(), 0.0) / leafDepths.size();
        float sq_sum = std::inner_product(leafDepths.begin(), leafDepths.end(), leafDepths.begin(), 0.0);
        float stdev = std::sqrt(sq_sum / leafDepths.size() - mean * mean);
        std::cout << "Standard Deviation of OBB Tree Leaf Depths: " << stdev << std::endl;

        std::cout << "Generating AABB BVH ... " << std::endl;

        start = std::chrono::high_resolution_clock::now();
        BVH bvh(triangles);
        end = std::chrono::high_resolution_clock::now();

        auto time_aabb_construction = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "AABB BVH Construction Time taken: " << time_aabb_construction << " ms" << std::endl;

        if (bvh.failed())
        {
            std::cerr << "BVH construction failed" << std::endl;
        }

        // calculate standard deviation of the leaf depths
        leafDepths = bvh.getLeafDepths();
        mean = std::accumulate(leafDepths.begin(), leafDepths.end(), 0.0) / leafDepths.size();
        sq_sum = std::inner_product(leafDepths.begin(), leafDepths.end(), leafDepths.begin(), 0.0);
        stdev = std::sqrt(sq_sum / leafDepths.size() - mean * mean);
        std::cout << "Standard Deviation of AABB BVH Leaf Depths: " << stdev << std::endl;
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

}