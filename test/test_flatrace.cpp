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

    // check if the scene if split correctly
//    TEST(FlatRace, BVH_Accuracy)
//    {
//        std::vector<std::string> fileCode = {"a", "b", "c", "d", "e", "f", "g"};
//
//        for (const std::string &code : fileCode)
//        {
//            std::cout << std::fixed << std::setprecision(2);
//            std::cout << "Processing file: " << code << std::endl;
//            std::string inputFile = code;
//
//            std::vector<std::vector<Triangle>> models;
//            models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Semi + inputFile, false);
//            std::vector<Triangle> triangles;
//
//            for (const auto &model : models)
//            {
//                triangles.insert(triangles.end(), model.begin(), model.end());
//            }
//
//            // AABB BVH without OBB
//            BVH bvh(triangles, false);
//            int aabbBVHErrorNodeCount = 0;
//            for (const auto &node : bvh.getNodes())
//            {
//                if (node.count > LEAF_SIZE)
//                {
//                    aabbBVHErrorNodeCount++;
//                }
//            }
//            std::cout << "AABB BVH (No OBB) Error Node Count: " << aabbBVHErrorNodeCount << std::endl;
//
//            // AABB BVH with OBB
//            BVH bvh_obb(triangles, true);
//            int aabbBVH_OBBErrorNodeCount = 0;
//            for (const auto &node : bvh_obb.getNodes())
//            {
//                if (node.count > LEAF_SIZE)
//                {
//                    aabbBVH_OBBErrorNodeCount++;
//                }
//            }
//            std::cout << "AABB BVH (Has OBB) Error Node Count: " << aabbBVH_OBBErrorNodeCount << std::endl;
//
//            // OBB tree Midpoint
//            core::obb::ObbTree obbTree(triangles, false, false);
//            int obbTreeErrorNodeCount = 0;
//            for (const auto &node : obbTree.getNodes())
//            {
//                if (node.count > LEAF_SIZE)
//                {
//                    obbTreeErrorNodeCount++;
//                }
//            }
//            std::cout << "OBB Tree (Midpoint) Error Node Count: " << obbTreeErrorNodeCount << std::endl;
//
//            // OBB tree SAH
//            core::obb::ObbTree obbTree_sah(triangles, true, false);
//            int obbTreeSAHErrorNodeCount = 0;
//            for (const auto &node : obbTree_sah.getNodes())
//            {
//                if (node.count > LEAF_SIZE)
//                {
//                    obbTreeSAHErrorNodeCount++;
//                }
//            }
//            std::cout << "OBB Tree (SAH) Error Node Count: " << obbTreeSAHErrorNodeCount << std::endl << std::endl;
//        }
//    }

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

            // AABB BVH without OBB
            auto start = std::chrono::high_resolution_clock::now();
            BVH bvh(triangles, false);
            auto end = std::chrono::high_resolution_clock::now();
            float time_aabb_construction = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            std::cout << "AABB BVH (No OBB) Construction Time taken: " << time_aabb_construction << " ms" << std::endl;

            // AABB BVH with OBB
            start = std::chrono::high_resolution_clock::now();
            BVH bvh_obb(triangles, true);
            end = std::chrono::high_resolution_clock::now();
            float time_aabb_withobb_construction = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            std::cout << "AABB BVH (Has OBB) Construction Time taken: " << time_aabb_withobb_construction << " ms"   << std::endl;;

            // OBB tree Midpoint
            start = std::chrono::high_resolution_clock::now();
            core::obb::ObbTree obbTree(triangles, false, false);
            end = std::chrono::high_resolution_clock::now();
            float time_obb_construction = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            std::cout << "OBB Tree (Midpoint) Construction Time taken: " << time_obb_construction << " ms" << std::endl;

            // OBB tree SAH
            start = std::chrono::high_resolution_clock::now();
            core::obb::ObbTree obbTree_sah(triangles, true, false);
            end = std::chrono::high_resolution_clock::now();
            float time_obb_sah_construction = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            std::cout << "OBB Tree (SAH) Construction Time taken: " << time_obb_sah_construction << " ms" << std::endl << std::endl;
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
                totalSAHCost += node.obb.area() * node.count;;
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
            totalSAHCost += node.obb.area()  * node.count;;
        }
        return totalSAHCost;
    }

    double calculateHybridTreeSAHCost(BVH &bvh)
    {
        std::vector<Node> nodes = bvh.getNodes();
        double totalSAHCost = 0.0;

        for (const auto &node : nodes)
        {
            if (node.obbFlag)
            {
                totalSAHCost += node.obb.area() * node.count;;
            } else
            {
                totalSAHCost += node.bbox.area() * node.count;;
            }
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

            // OBB tree Midpoint
            core::obb::ObbTree obbTree(triangles, false, false);
            sahCost = CalculateOBBTreeSAHCost(obbTree);
            std::cout << "SAH Cost of OBB Tree (Midpoint): " << sahCost << std::endl;

            // OBB tree SAH
            core::obb::ObbTree obbTree_sah(triangles, true, false);
            sahCost = CalculateOBBTreeSAHCost(obbTree_sah);
            std::cout << "SAH Cost of OBB Tree (SAH): " << sahCost << std::endl;
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

            // OBB tree Midpoint
            core::obb::ObbTree obbTree(triangles, false, false);
            leafDepths = obbTree.getLeafDepths();
            mean = std::accumulate(leafDepths.begin(), leafDepths.end(), 0.0) / leafDepths.size();
            sq_sum = std::inner_product(leafDepths.begin(), leafDepths.end(), leafDepths.begin(), 0.0);
            stddev = std::sqrt(sq_sum / leafDepths.size() - mean * mean);
            std::cout << "Standard Deviation of OBB Tree (Midpoint) Leaf Depths: " << stddev << std::endl;

            // OBB tree SAH
            core::obb::ObbTree obbTree_sah(triangles, true, false);
            leafDepths = obbTree_sah.getLeafDepths();
            mean = std::accumulate(leafDepths.begin(), leafDepths.end(), 0.0) / leafDepths.size();
            sq_sum = std::inner_product(leafDepths.begin(), leafDepths.end(), leafDepths.begin(), 0.0);
            stddev = std::sqrt(sq_sum / leafDepths.size() - mean * mean);
            std::cout << "Standard Deviation of OBB Tree (SAH) Leaf Depths: " << stddev << std::endl << std::endl;
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

            // AABB without OBB
            BVH bvh(triangles, false);

            // AABB with OBB
            BVH bvh_obb(triangles, true);

            // OBB Midpoint
            core::obb::ObbTree obbTree(triangles, false, false);

            // OBB SAH
            core::obb::ObbTree obbTreeSAH(triangles, true, false);

            // Hybrid tree
            BVH bvh_hybrid(triangles, true);

            Frame frame(FRAME_WIDTH, FRAME_HEIGHT);

            double totalTimeAABB = 0.0;
            double totalTimeAABB_OBB = 0.0;
            double totalTimeOBB = 0.0;
            double totalTimeOBBSAH = 0.0;

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

                // OBB SAH
                start = std::chrono::high_resolution_clock::now();
                render_frameOBB(camera, obbTreeSAH, frame.pixels.get(), false, false);
                end = std::chrono::high_resolution_clock::now();
                auto time_obb_sah = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                totalTimeOBBSAH += time_obb_sah;

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
            std::cout << "Average Render Semiconductor OBB (Midpoint) Time taken: " << totalTimeOBB / 10.0 << " ms"
                      << std::endl;
            std::cout << "Average Render Semiconductor OBB (SAH) Time taken: " << totalTimeOBBSAH / 10.0 << " ms"
                    << std::endl << std::endl;
        }
    }

    // Test basics of Hybrid Tree using different offset numbers
    TEST(FlatRace, BVH_Hybrid)
    {
        std::vector<std::string> fileCode = {"a"};
        std::vector<float> offsets = {0.01, 0.05, 0.1, 0.5, 1, 1.5, 2};

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

            for (const float num: offsets)
            {
                auto start = std::chrono::high_resolution_clock::now();
                core::BVH hybrid(triangles, true, num);
                auto end = std::chrono::high_resolution_clock::now();
                auto time_hybrid_build = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                std::cout << "Hybrid Tree (Offset: " << num << ") Construction Time taken: " << time_hybrid_build << " ms" << std::endl;

                // SAH
                double sahCost = calculateHybridTreeSAHCost(hybrid);
                std::cout << "SAH Cost of Hybrid Tree (Offset: " << num << "): " << sahCost << std::endl;

                // Render (No caching)
                Frame frame(FRAME_WIDTH, FRAME_HEIGHT);
                double totalTime = 0.0;

                for (int i = 0; i < 10; ++i)
                {
                    float angle = 2 * PI * i / 10;
                    float cx = std::cos(angle) * 2.0f;
                    float cz = std::sin(angle) * 2.0f;
                    Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};

                    // AABB with OBB
                    auto start = std::chrono::high_resolution_clock::now();
                    render_frameHybrid(camera, hybrid, frame.pixels.get(), false);
                    auto end = std::chrono::high_resolution_clock::now();
                    auto time_hybrid = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                    totalTime += time_hybrid;
                }
                std::cout << "Average Render Hybrid Tree (Offset: " << num << ") Time taken: " << totalTime / 10.0 << " ms" << std::endl;

                // Render (caching)
                Frame frame_caching(FRAME_WIDTH, FRAME_HEIGHT);
                double totalTime_caching = 0.0;

                for (int i = 0; i < 10; ++i)
                {
                    float angle = 2 * PI * i / 10;
                    float cx = std::cos(angle) * 2.0f;
                    float cz = std::sin(angle) * 2.0f;
                    Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};

                    // AABB with OBB
                    auto start = std::chrono::high_resolution_clock::now();
                    render_frameHybrid(camera, hybrid, frame.pixels.get(), false);
                    auto end = std::chrono::high_resolution_clock::now();
                    auto time_hybrid = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                    totalTime_caching += time_hybrid;
                }
                std::cout << "Average Render Hybrid Tree(caching) (Offset: " << num << ") Time taken: " << totalTime_caching / 10.0 << " ms" << std::endl << std::endl;

            }
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
            core::obb::ObbTree obbTree(triangles, true, false);

            Frame frame(FRAME_WIDTH, FRAME_HEIGHT);

            double totalTimeOBB = 0.0;

            for (int i = 0; i < 10; ++i)
            {
                float angle = 2 * PI * i / 10;
                float cx = std::cos(angle) * 2.0f;
                float cz = std::sin(angle) * 2.0f;
                Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};

                // OBB without clustering / with raycaching
                auto start = std::chrono::high_resolution_clock::now();
                render_frameOBB(camera, obbTree, frame.pixels.get(), false, true);
                auto end = std::chrono::high_resolution_clock::now();
                auto time_obb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                totalTimeOBB += time_obb;
            }

            std::cout << "Average Render Semiconductor OBB (Midpoint, No Clustering, With Caching) Time taken: " << totalTimeOBB / 10.0 << " ms"
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
                core::obb::ObbTree obbTree(triangles, true, true, 32, num);
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
                core::obb::ObbTree obbTree(triangles, true, true, 32, num);
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