#include <gtest/gtest.h>
#include <numeric>

#include "src/core/types.h"
#include "src/core/bvh.h"
#include "src/core/obbTree.h"

#include "src/core/trace.h"
#include "src/utils/obj.h"
#include "src/utils/ppm.h"

constexpr float PI = 3.14159265359f;

using namespace core;

namespace test
{
    const std::string TEST_OBJ = "test/input/house_interior.obj";
    const std::string TEST_OBJ_FOLDER_Bunny = "test/input/test";
    const std::string TEST_OBJ_FOLDER_Semi = "test/input/stacks/stack_";

//{
//    //  test :loading a simple obj file
//    TEST(FlatRace, ObjLoading_1)
//    {
//        std::vector<Triangle> test_model = utils::Obj::read(TEST_OBJ, true);
//    }
//
//    // test : loading multiple obj files
//    TEST(FlatRace, ObjLoading_2)
//    {
//        std::vector<std::vector<Triangle>> models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Semi, false);
//    }
//
//    //  test :simple test the construction of the aabb BVH
//    TEST(FlatRace, Construction_1)
//    {
//        const std::vector<std::pair<int, glm::vec3>> POINTS = {
//                {0, {-20.0f, -20.0f, 0.0f}},
//                {1, {11.0f,  21.0f,  0.0f}},
//                {2, {-20.0f, -10.0f, 0.0f}},
//                {3, {21.0f,  11.0f,  0.0f}},
//                {4, {12.0f,  -12.0f, 42.0f}},
//                {5, {-10.0f, -20.0f, 0.0f}},
//        };
//
//        std::vector<Triangle> triangles(POINTS.size());
//        std::transform(POINTS.begin(), POINTS.end(), triangles.begin(),
//                       [](const auto &v) -> Triangle { return {v.first, v.second, v.second, v.second, 0}; });
//
//        BVH bvh(triangles);
//        if (bvh.failed())
//        {
//            std::cerr << "BVH construction failed" << std::endl;
//        }
//
//        Frame frame(FRAME_WIDTH, FRAME_HEIGHT);
//
//        // camera
//        const float cx = std::cos(0.0f) * 2.0f;
//        const float cz = std::sin(0.0f) * 2.0f;
//        Camera camera = {{cx, 1.0f, cz},
//                         {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};
//        int maxDepth = bvh.getMaxDepth();
//        render_frame(camera, bvh, frame.pixels.get(), maxDepth);
//    }
//
    //  test :compare obb tree, aabb, aabb with obb (bunny)
    TEST(FlatRace, Render_1)
    {
        std::vector<std::vector<core::Triangle>> models;
        models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Bunny, true);
        std::vector<core::Triangle> triangles;
        for (const auto &model : models)
        {
            triangles.insert(triangles.end(), model.begin(), model.end());
        }

        core::obb::ObbTree obbTree(triangles, false);

        BVH bvh(triangles, false);
        BVH bvhOBB(triangles, true);

        Frame frame(FRAME_WIDTH, FRAME_HEIGHT);

        double totalTimeAABB = 0.0;
        double totalTimeAABB_OBB = 0.0;
        double totalTimeOBB = 0.0;

        for (int i = 0; i < 10; ++i)
        {
            float angle = 2 * PI * i / 10;
            float cx = std::cos(angle) * 2.0f;
            float cz = std::sin(angle) * 2.0f;
            Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};

            auto start = std::chrono::high_resolution_clock::now();
            render_frameOBB(camera, obbTree, frame.pixels.get(), false, false);
            auto end = std::chrono::high_resolution_clock::now();
            auto time_obb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            totalTimeOBB += time_obb;

            start = std::chrono::high_resolution_clock::now();
            render_frame(camera, bvh, frame.pixels.get(), false, false);
            end = std::chrono::high_resolution_clock::now();
            auto time_aabb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            totalTimeAABB += time_aabb;

            start = std::chrono::high_resolution_clock::now();
            render_frame(camera, bvhOBB, frame.pixels.get(), true, false);
            end = std::chrono::high_resolution_clock::now();
            auto time_aabb_obb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            totalTimeAABB_OBB += time_aabb_obb;
        }

        std::cout << "Average Render Bunny AABB Time taken: " << totalTimeAABB / 10.0 << " ms" << std::endl;
        std::cout << "Average Render Bunny OBB in AABB Time taken: " << totalTimeAABB_OBB / 10.0 << " ms" << std::endl;
        std::cout << "Average Render Bunny OBB in OBB Tree Time taken: " << totalTimeOBB / 10.0 << " ms" << std::endl;
    }
//
//    //  test :compare obb tree, aabb, aabb with obb (semiconductor)
//    TEST(FlatRace, Render_2)
//    {
//        std::vector<std::vector<core::Triangle>> models;
//        models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Semi, false);
//        std::vector<core::Triangle> triangles;
//        for (const auto &model : models) {
//            triangles.insert(triangles.end(), model.begin(), model.end());
//        }
//
//        core::obb::ObbTree obbTree(triangles);
//        if (obbTree.failed())
//        {
//            std::cerr << "obbTree construction failed" << std::endl;
//        }
//        BVH bvh(triangles);
//
//        Frame frame(FRAME_WIDTH, FRAME_HEIGHT);
//
//        double totalTimeAABB = 0.0;
//        double totalTimeAABB_OBB = 0.0;
//        double totalTimeOBB = 0.0;
//
//        for (int i = 0; i < 10; ++i)
//        {
//            float angle = 2 * PI * i / 10;
//            float cx = std::cos(angle) * 2.0f;
//            float cz = std::sin(angle) * 2.0f;
//            Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};
//
//            auto start = std::chrono::high_resolution_clock::now();
//            render_frameOBB(camera, obbTree, frame.pixels.get());
//            auto end = std::chrono::high_resolution_clock::now();
//            auto time_obb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
//            totalTimeOBB += time_obb;
//
//            start = std::chrono::high_resolution_clock::now();
//            render_frame(camera, bvh, frame.pixels.get(), false);
//            end = std::chrono::high_resolution_clock::now();
//            auto time_aabb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
//            totalTimeAABB += time_aabb;
//
//            start = std::chrono::high_resolution_clock::now();
//            render_frame(camera, bvh, frame.pixels.get(), true);
//            end = std::chrono::high_resolution_clock::now();
//            auto time_aabb_obb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
//            totalTimeAABB_OBB += time_aabb_obb;
//        }
//
//        std::cout << "Average Render Semiconductor AABB Time taken: " << totalTimeAABB / 10.0 << " ms" << std::endl;
//        std::cout << "Average Render Semiconductor OBB in AABB Time taken: " << totalTimeAABB_OBB / 10.0 << " ms" << std::endl;
//        std::cout << "Average Render Semiconductor OBB in OBB Tree Time taken: " << totalTimeOBB / 10.0 << " ms" << std::endl;
//    }
//
//    // Profiling tests
//    //  test bvh
//    TEST(FlatRace, BVH_1)
//    {
//        std::vector<std::vector<Triangle>> models;
//        models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Semi, false);
//        std::vector<Triangle> triangles;
//        for (const auto &model : models)
//        {
//            triangles.insert(triangles.end(), model.begin(), model.end());
//        }
//
//        std::cout << "Generating OBB Tree ... " << std::endl;
//        const float cx = std::cos(0.0f) * 2.0f;
//        const float cz = std::sin(0.0f) * 2.0f;
//        Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};
//        auto start = std::chrono::high_resolution_clock::now();
//        core::obb::ObbTree obbTree(triangles);
//        auto end = std::chrono::high_resolution_clock::now();
//        auto time_obb_construction = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
//        std::cout << "OBB Tree Construction Time taken: " << time_obb_construction << " ms" << std::endl;
//        if (obbTree.failed())
//        {
//            std::cerr << "obbTree construction failed" << std::endl;
//        }
//
//        // calculate standard deviation of the leaf depths
//        std::vector<int> leafDepths = obbTree.getLeafDepths();
//        float mean = std::accumulate(leafDepths.begin(), leafDepths.end(), 0.0) / leafDepths.size();
//        float sq_sum = std::inner_product(leafDepths.begin(), leafDepths.end(), leafDepths.begin(), 0.0);
//        float stdev = std::sqrt(sq_sum / leafDepths.size() - mean * mean);
//        std::cout << "Standard Deviation of OBB Tree Leaf Depths: " << stdev << std::endl;
//
//        std::cout << "Generating AABB BVH ... " << std::endl;
//
//        start = std::chrono::high_resolution_clock::now();
//        BVH bvh(triangles);
//        end = std::chrono::high_resolution_clock::now();
//
//        auto time_aabb_construction = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
//        std::cout << "AABB BVH Construction Time taken: " << time_aabb_construction << " ms" << std::endl;
//
//        if (bvh.failed())
//        {
//            std::cerr << "BVH construction failed" << std::endl;
//        }
//
//        // calculate standard deviation of the leaf depths
//        leafDepths = bvh.getLeafDepths();
//        mean = std::accumulate(leafDepths.begin(), leafDepths.end(), 0.0) / leafDepths.size();
//        sq_sum = std::inner_product(leafDepths.begin(), leafDepths.end(), leafDepths.begin(), 0.0);
//        stdev = std::sqrt(sq_sum / leafDepths.size() - mean * mean);
//        std::cout << "Standard Deviation of AABB BVH Leaf Depths: " << stdev << std::endl;
//    }
//
//    // TODO: calculate surface area of the obb tree and aabb bvh
//    TEST(FlatRace, BVH_2)
//    {
//    }
//
//    // Test clustering vs no-clustering (OBB BVH)
//    TEST(FlatRace, Clustering_1)
//    {
//        std::vector<std::vector<core::Triangle>> models;
//        models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Semi, false);
//        std::vector<core::Triangle> triangles;
//        for (const auto &model : models) {
//            triangles.insert(triangles.end(), model.begin(), model.end());
//        }
//
//        core::obb::ObbTree obbTree(triangles);
//        if (obbTree.failed())
//        {
//            std::cerr << "obbTree construction failed" << std::endl;
//        }
//
//        Frame frame(FRAME_WIDTH, FRAME_HEIGHT);
//
//        double totalTimeClustering = 0.0;
//        double totalTimeNonClustering = 0.0;
//
//        for (int i = 0; i < 10; ++i)
//        {
//            float angle = 2 * PI * i / 10;
//            float cx = std::cos(angle) * 2.0f;
//            float cz = std::sin(angle) * 2.0f;
//            Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};
//
//            auto start = std::chrono::high_resolution_clock::now();
//            render_frameOBB(camera, obbTree, frame.pixels.get(), true);
//            auto end = std::chrono::high_resolution_clock::now();
//            auto time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
//            totalTimeClustering += time;
//
//            start = std::chrono::high_resolution_clock::now();
//            render_frameOBB(camera, obbTree, frame.pixels.get(), false);
//            end = std::chrono::high_resolution_clock::now();
//            time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
//            totalTimeNonClustering += time;
//
//        }
//
//        std::cout << "Average Render Semiconductor OBB in OBB Tree with Clustering Time taken: " << totalTimeClustering / 10.0 << " ms" << std::endl;
//        std::cout << "Average Render Semiconductor OBB in OBB Tree without Clustering Time taken: " << totalTimeNonClustering / 10.0 << " ms" << std::endl;
//    }
//}

    // Tests for getting data
    // Models' triangle count
    TEST(FlatRace, Analysis_Models_Triangle_Count)
    {
        std::vector<std::string> fileCode = {"a", "b", "c", "d", "e", "f", "g"};

        for (const std::string &code : fileCode)
        {
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Processing file: " << code << std::endl;
            std::string inputFile = code;

            std::vector<std::vector<Triangle>> models;
            models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Semi + inputFile, false);
            std::vector<Triangle> triangles;

            for (const auto &model : models)
            {
                triangles.insert(triangles.end(), model.begin(), model.end());
            }

            std::cout << "Triangle count: " << triangles.size() << std::endl;
        }
    }

    // BVH building time
    TEST(FlatRace, Analysis_BVH_Build_time)
    {
        std::vector<std::string> fileCode = {"a", "b", "c", "d", "e", "f", "g"};

        for (const std::string &code : fileCode)
        {
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Processing file: " << code << std::endl;
            std::string inputFile = code;

            std::vector<std::vector<Triangle>> models;
            models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Semi + inputFile, false);
            std::vector<Triangle> triangles;

            for (const auto &model : models)
            {
                triangles.insert(triangles.end(), model.begin(), model.end());
            }

            // OBB tree without clustering
            auto start = std::chrono::high_resolution_clock::now();
            core::obb::ObbTree obbTree(triangles, false);
            auto end = std::chrono::high_resolution_clock::now();
            float time_obb_construction = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            std::cout << "OBB Tree (No Clustering) Construction Time taken: " << time_obb_construction << " ms" << std::endl;

            // AABB BVH without OBB
            start = std::chrono::high_resolution_clock::now();
            BVH bvh(triangles, false);
            end = std::chrono::high_resolution_clock::now();
            float time_aabb_construction = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            std::cout << "AABB BVH (No OBB) Construction Time taken: " << time_aabb_construction << " ms" << std::endl;

            // AABB BVH with OBB
            start = std::chrono::high_resolution_clock::now();
            BVH bvh_obb(triangles, true);
            end = std::chrono::high_resolution_clock::now();
            float time_aabb_withobb_construction = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            std::cout << "AABB BVH (Has OBB) Construction Time taken: " << time_aabb_withobb_construction << " ms" << std::endl  << std::endl;;
        }
    }

    // BVH Metrics
    double calculateAABBTreeSAHCost(BVH &bvh, bool isOBB)
    {
        std::vector<Node> nodes = bvh.getNodes();
        double totalSAHCost = 0.0;

        if (!isOBB)
        {
            for (const auto &node: nodes)
            {
                // DO not consider the child nodes
                    totalSAHCost += node.bbox.area() * node.count;
            }
        } else
        {
            for (const auto &node: nodes)
            {
                totalSAHCost += node.obb.area() * node.count;
            }
        }

        return totalSAHCost;
    }

    double CalculateOBBTreeSAHCost(core::obb::ObbTree &obbTree)
    {
        std::vector<core::obb::Node> nodes = obbTree.getNodes();
        double totalSAHCost = 0.0;
        for (const auto &node : nodes)
        {
            // DO not consider the child nodes
            totalSAHCost += node.obb.area() * node.count;
        }
        return totalSAHCost;
    }

    TEST(FlatRace, BVH_Mtrics_SAHCost)
    {
        std::vector<std::string> fileCode = {"a", "b", "c", "d", "e", "f", "g"};

        for (const std::string &code : fileCode)
        {
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Processing file: " << code << std::endl;
            std::string inputFile = code;

            std::vector<std::vector<Triangle>> models;
            models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Semi + inputFile, false);
            std::vector<Triangle> triangles;

            for (const auto &model : models)
            {
                triangles.insert(triangles.end(), model.begin(), model.end());
            }

            // AABB BVH without OBB
            BVH bvh(triangles, false);
            double sahCost = calculateAABBTreeSAHCost(bvh, false);
            std::cout << "SAH Cost of AABB (No OBB) BVH: " << sahCost << std::endl;

            // AABB BVH with OBB
            BVH bvh_obb(triangles, true);
            sahCost = calculateAABBTreeSAHCost(bvh_obb, true);
            std::cout << "SAH Cost of AABB (Has OBB) BVH: " << sahCost << std::endl;

            // OBB tree without clustering
            core::obb::ObbTree obbTree(triangles, false);
            sahCost = CalculateOBBTreeSAHCost(obbTree);
            std::cout << "SAH Cost of OBB Tree (No Clustering): " << sahCost << std::endl << std::endl;

        }
    }

    TEST(FlatRace, BVH_Mtrics_hitratio)
    {
        std::vector<std::string> fileCode = {"a", "b", "c", "d", "e", "f", "g"};

        for (const std::string &code : fileCode)
        {
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Processing file: " << code << std::endl;
            std::string inputFile = code;

            std::vector<std::vector<Triangle>> models;
            models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Semi + inputFile, false);
            std::vector<Triangle> triangles;

            for (const auto &model : models)
            {
                triangles.insert(triangles.end(), model.begin(), model.end());
            }

            // AABB BVH without OBB
            BVH bvh(triangles, false);

            // AABB BVH with OBB
            BVH bvh_obb(triangles, true);

            // OBB tree without clustering
            core::obb::ObbTree obbTree(triangles, false);
        }
    }

    TEST(FlatRace, BVH_Mtrics_stddev)
    {
        std::vector<std::string> fileCode = {"a", "b", "c", "d", "e", "f", "g"};

        for (const std::string &code : fileCode)
        {
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Processing file: " << code << std::endl;
            std::string inputFile = code;

            std::vector<std::vector<Triangle>> models;
            models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Semi + inputFile, false);
            std::vector<Triangle> triangles;

            for (const auto &model : models)
            {
                triangles.insert(triangles.end(), model.begin(), model.end());
            }

            // AABB BVH without OBB
            BVH bvh(triangles, false);
            std::vector<int> leafDepths = bvh.getLeafDepths();
            double mean = std::accumulate(leafDepths.begin(), leafDepths.end(), 0.0) / leafDepths.size();
            double sq_sum = std::inner_product(leafDepths.begin(), leafDepths.end(), leafDepths.begin(), 0.0);
            double stddev = std::sqrt(sq_sum / leafDepths.size() - mean * mean);
            std::cout << "Standard Deviation of AABB (No OBB) BVH Leaf Depths: " << stddev << std::endl;

            // AABB BVH with OBB
            BVH bvh_obb(triangles, true);
            leafDepths = bvh_obb.getLeafDepths();
            mean = std::accumulate(leafDepths.begin(), leafDepths.end(), 0.0) / leafDepths.size();
            sq_sum = std::inner_product(leafDepths.begin(), leafDepths.end(), leafDepths.begin(), 0.0);
            stddev = std::sqrt(sq_sum / leafDepths.size() - mean * mean);
            std::cout << "Standard Deviation of AABB (Has OBB) BVH Leaf Depths: " << stddev << std::endl;

            // OBB tree without clustering
            core::obb::ObbTree obbTree(triangles, false);
            leafDepths = obbTree.getLeafDepths();
            mean = std::accumulate(leafDepths.begin(), leafDepths.end(), 0.0) / leafDepths.size();
            sq_sum = std::inner_product(leafDepths.begin(), leafDepths.end(), leafDepths.begin(), 0.0);
            stddev = std::sqrt(sq_sum / leafDepths.size() - mean * mean);
            std::cout << "Standard Deviation of OBB Tree (No Clustering) Leaf Depths: " << stddev << std::endl << std::endl;
        }
    }

    // Basic render time
    TEST(FlatRace, Analysis_Render_time)
    {
        std::vector<std::string> fileCode = {"a", "b", "c", "d", "e", "f", "g"};

        for (const std::string &code : fileCode)
        {
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Processing file: " << code << std::endl;
            std::string inputFile = code;

            std::vector<std::vector<core::Triangle>> models;
            models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Semi + code, false);
            std::vector<core::Triangle> triangles;
            for (const auto &model: models)
            {
                triangles.insert(triangles.end(), model.begin(), model.end());
            }

            // OBB without clustering
            core::obb::ObbTree obbTree(triangles, false);

            // AABB without OBB
            BVH bvh(triangles, false);

            // AABB with OBB
            BVH bvh_obb(triangles, true);

            Frame frame(FRAME_WIDTH, FRAME_HEIGHT);

            double totalTimeAABB = 0.0;
            double totalTimeAABB_OBB = 0.0;
            double totalTimeOBB = 0.0;

            for (int i = 0; i < 10; ++i)
            {
                float angle = 2 * PI * i / 10;
                float cx = std::cos(angle) * 2.0f;
                float cz = std::sin(angle) * 2.0f;
                Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};

                // OBB without clustering / without raycaching
                auto start = std::chrono::high_resolution_clock::now();
                render_frameOBB(camera, obbTree, frame.pixels.get(), false, false);
                auto end = std::chrono::high_resolution_clock::now();
                auto time_obb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                totalTimeOBB += time_obb;

                // AABB without OBB
                start = std::chrono::high_resolution_clock::now();
                render_frame(camera, bvh, frame.pixels.get(), false, false);
                end = std::chrono::high_resolution_clock::now();
                auto time_aabb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                totalTimeAABB += time_aabb;

                // AABB with OBB
                start = std::chrono::high_resolution_clock::now();
                render_frame(camera, bvh_obb, frame.pixels.get(), true, false);
                end = std::chrono::high_resolution_clock::now();
                auto time_aabb_obb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                totalTimeAABB_OBB += time_aabb_obb;
            }

            std::cout << "Average Render Semiconductor AABB (No OBB) Time taken: " << totalTimeAABB / 10.0 << " ms"
                      << std::endl;
            std::cout << "Average Render Semiconductor AABB (Use OBB) Time taken: " << totalTimeAABB_OBB / 10.0 << " ms"
                      << std::endl;
            std::cout << "Average Render Semiconductor OBB (No Clustering) Time taken: " << totalTimeOBB / 10.0 << " ms"
                      << std::endl << std::endl;
        }
    }

    // Testing ray caching
    TEST(FlatRace, Caching_Render_Time_AABBTree)
    {
        std::vector<std::string> fileCode = {"a", "b", "c", "d", "e", "f", "g"};

        for (const std::string &code : fileCode)
        {
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Processing file: " << code << std::endl;
            std::string inputFile = code;

            std::vector<std::vector<core::Triangle>> models;
            models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Semi + code, false);
            std::vector<core::Triangle> triangles;
            for (const auto &model: models)
            {
                triangles.insert(triangles.end(), model.begin(), model.end());
            }

            // AABB with OBB
            BVH bvh_obb(triangles, true);
            Frame frame(FRAME_WIDTH, FRAME_HEIGHT);

            double totalTimeOBB = 0.0;

            for (int i = 0; i < 10; ++i)
            {
                float angle = 2 * PI * i / 10;
                float cx = std::cos(angle) * 2.0f;
                float cz = std::sin(angle) * 2.0f;
                Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};

                //caching
                auto start = std::chrono::high_resolution_clock::now();
                render_frame(camera, bvh_obb, frame.pixels.get(), true, true);
                auto end = std::chrono::high_resolution_clock::now();
                auto time_obb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                totalTimeOBB += time_obb;
            }

            std::cout << "Average Render Semiconductor AABB Tree(With OBB, With Caching) Time taken: " << totalTimeOBB / 10.0 << " ms"
                      << std::endl << std::endl;
        }
    }

    TEST(FlatRace, Caching_Render_Time_OBBTree)
    {
        std::vector<std::string> fileCode = {"a", "b", "c", "d", "e", "f", "g"};

        for (const std::string &code : fileCode)
        {
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Processing file: " << code << std::endl;
            std::string inputFile = code;

            std::vector<std::vector<core::Triangle>> models;
            models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Semi + code, false);
            std::vector<core::Triangle> triangles;
            for (const auto &model: models)
            {
                triangles.insert(triangles.end(), model.begin(), model.end());
            }

            // OBB without clustering
            core::obb::ObbTree obbTree(triangles, false);

            Frame frame(FRAME_WIDTH, FRAME_HEIGHT);

            double totalTimeOBB = 0.0;

            for (int i = 0; i < 10; ++i)
            {
                float angle = 2 * PI * i / 10;
                float cx = std::cos(angle) * 2.0f;
                float cz = std::sin(angle) * 2.0f;
                Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};

                // OBB without clustering / without raycaching
                auto start = std::chrono::high_resolution_clock::now();
                render_frameOBB(camera, obbTree, frame.pixels.get(), false, true);
                auto end = std::chrono::high_resolution_clock::now();
                auto time_obb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                totalTimeOBB += time_obb;
            }

            std::cout << "Average Render Semiconductor OBB (No Clustering, With Caching) Time taken: " << totalTimeOBB / 10.0 << " ms"
                      << std::endl << std::endl;
        }
    }

    // Test clustering
    TEST(FlatRace, Build_Clustering)
    {
        std::vector<std::string> fileCode = {"a"};
        std::vector<int> k = {1 , 10 , 100 , 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000};

        for (const std::string &code : fileCode)
        {
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Processing file: " << code << std::endl;
            std::string inputFile = code;

            for (const int num: k)
            {
                std::vector<std::vector<Triangle>> models;
                models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Semi + inputFile, false);
                std::vector<Triangle> triangles;

                for (const auto &model: models)
                {
                    triangles.insert(triangles.end(), model.begin(), model.end());
                }

                // OBB tree without clustering
                auto start = std::chrono::high_resolution_clock::now();
                core::obb::ObbTree obbTree(triangles, true, num);
                auto end = std::chrono::high_resolution_clock::now();
                float time_obb_construction = std::chrono::duration_cast<std::chrono::milliseconds>(
                        end - start).count();
                std::cout << "OBB Tree: k =  " << num << " Clustering, Construction Time taken: " << time_obb_construction << " ms"
                          << std::endl;
            }
        }
    }

    TEST(FlatRace, Render_KMeans)
    {
        std::vector<std::string> fileCode = {"a"};
        std::vector<int> k = {1 , 10 , 100 , 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000};

        for (const std::string &code : fileCode)
        {
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Processing file: " << code << std::endl;
            std::string inputFile = code;

            std::vector<std::vector<core::Triangle>> models;
            models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Semi + code, false);
            std::vector<core::Triangle> triangles;
            for (const auto &model: models)
            {
                triangles.insert(triangles.end(), model.begin(), model.end());
            }

            for (const int num: k)
            {
                core::obb::ObbTree obbTree(triangles, true, num);
                double sahCost = CalculateOBBTreeSAHCost(obbTree);
                std::cout << "SAH Cost of OBB Tree (Clustering): " << sahCost << std::endl << std::endl;

                Frame frame(FRAME_WIDTH, FRAME_HEIGHT);

                double totalTimeOBB = 0.0;

                for (int i = 0; i < 10; ++i)
                {
                    float angle = 2 * PI * i / 10;
                    float cx = std::cos(angle) * 2.0f;
                    float cz = std::sin(angle) * 2.0f;
                    Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};

                    // OBB without clustering / without raycaching
                    auto start = std::chrono::high_resolution_clock::now();
                    render_frameOBB(camera, obbTree, frame.pixels.get(), true, true);
                    auto end = std::chrono::high_resolution_clock::now();
                    auto time_obb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                    totalTimeOBB += time_obb;
                }

                std::cout << "Average Render Semiconductor OBB (Clustering) k = " << num << " Time taken: " << totalTimeOBB / 10.0
                          << " ms"
                          << std::endl << std::endl;
            }
        }
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