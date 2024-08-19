#include <gtest/gtest.h>
#include <numeric>
#include <chrono>
#include <iomanip>
#include <fstream>
#include <filesystem>
#include <iostream>

#include "src/core/types.h"
#include "src/core/bvh.h"
#include "src/core/obbTree.h"

#include "src/core/trace.h"
#include "src/utils/obj.h"
#include "src/utils/ppm.h"

constexpr float PI = 3.14159265359f;

using namespace core;
namespace fs = std::filesystem;

namespace test
{
    const std::string TEST_OBJ = "test/input/house_interior.obj";
    const std::string TEST_OBJ_FOLDER_Bunny = "test/input/test";
    const std::string TEST_OBJ_FOLDER_Semi = "test/input/stacks/stack_";

    void saveToCSV(const std::string& folder, const std::string& filename,
                   const std::string& header, const std::vector<std::pair<std::string, double>>& data)
    {
        fs::create_directories(folder); // Create directory if it doesn't exist
        std::ofstream file(folder + "/" + filename);
        file << header << "\n"; // CSV header
        for (const auto& entry : data)
        {
            file << entry.first << ", " << entry.second << "\n";
        }
    }

    // Models' triangle count
    TEST(FlatRace, Analysis_Models_Triangle_Count)
    {
        std::vector<std::string> fileCode = {"a", "b", "c", "d", "e", "f", "g"};
        std::vector<std::pair<std::string, double>> results;

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
            results.emplace_back(inputFile, static_cast<double>(triangles.size()));
        }

        // Save results to CSV with a custom header
        saveToCSV("results/Analysis_Models_Triangle_Count", "triangle_counts.csv", "Model, Triangle Count", results);
    }

    // Test BVH split accuracy
    TEST(FlatRace, BVH_Split_accuracy)
    {
        std::vector<std::string> fileCode = {"a", "b", "c", "d", "e", "f", "g"};

        for (const std::string &code: fileCode)
        {
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Processing file: " << code << std::endl;
            std::string inputFile = code;

            std::vector<std::vector<Triangle>> models;
            models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Semi + inputFile, false);
            std::vector<Triangle> triangles;

            for (const auto &model: models)
            {
                triangles.insert(triangles.end(), model.begin(), model.end());
            }

            // AABB BVH without OBB
            BVH bvh(triangles, false, false);

            // AABB BVH with OBB
            BVH bvh_obb(triangles, true, false);

            // OBB tree Midpoint
            core::obb::ObbTree obbTree_mid(triangles, false, false);

            // OBB tree Median
            core::obb::ObbTree obbTree_median(triangles, false, false, 16, 0, true);

            // OBB tree SAH
            core::obb::ObbTree obbTree_sah(triangles, true, false);

            // Hybird tree
            core::BVH hybrid(triangles, true, true);

            // Loop over each node
            for (const auto& node : bvh.getNodes())
            {
                if (node.count > TracerState::LEAF_SIZE)
                {
                    std::cout << "Wrong split found: " << node.count  << std::endl;
                }
            }
        }
    }

    // BVH building time
    TEST(FlatRace, Analysis_BVH_Build_time)
    {
        std::vector<std::string> fileCode = {"a", "b", "c", "d", "e", "f", "g"};

        for (const std::string &code: fileCode)
        {
            std::vector<std::pair<std::string, double>> results;
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Processing file: " << code << std::endl;
            std::string inputFile = code;

            std::vector<std::vector<Triangle>> models;
            models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Semi + inputFile, false);
            std::vector<Triangle> triangles;

            for (const auto &model: models)
            {
                triangles.insert(triangles.end(), model.begin(), model.end());
            }

            // AABB BVH without OBB
            auto start = std::chrono::high_resolution_clock::now();
            BVH bvh(triangles, false, false);
            auto end = std::chrono::high_resolution_clock::now();
            float time_aabb_construction = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            std::cout << "AABB BVH (No OBB) Construction Time taken: " << time_aabb_construction << " ms" << std::endl;

            // AABB BVH with OBB
            start = std::chrono::high_resolution_clock::now();
            BVH bvh_obb(triangles, true, false);
            end = std::chrono::high_resolution_clock::now();
            float time_aabb_withobb_construction = std::chrono::duration_cast<std::chrono::milliseconds>(
                    end - start).count();
            std::cout << "AABB BVH (Has OBB) Construction Time taken: " << time_aabb_withobb_construction << " ms"
                      << std::endl;;

            // OBB tree Midpoint
            start = std::chrono::high_resolution_clock::now();
            core::obb::ObbTree obbTree(triangles, false, false);
            end = std::chrono::high_resolution_clock::now();
            float time_obb_construction = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            std::cout << "OBB Tree (Midpoint) Construction Time taken: " << time_obb_construction << " ms" << std::endl;

            // OBB tree Median
            start = std::chrono::high_resolution_clock::now();
            core::obb::ObbTree obbTree_median(triangles, false, false, 16, 0, true);
            end = std::chrono::high_resolution_clock::now();
            float time_obb_median_construction = std::chrono::duration_cast<std::chrono::milliseconds>(
                    end - start).count();
            std::cout << "OBB Tree (Median) Construction Time taken: " << time_obb_median_construction << " ms"
                      << std::endl;

            // OBB tree SAH
            start = std::chrono::high_resolution_clock::now();
            core::obb::ObbTree obbTree_sah(triangles, true, false);
            end = std::chrono::high_resolution_clock::now();
            float time_obb_sah_construction = std::chrono::duration_cast<std::chrono::milliseconds>(
                    end - start).count();
            std::cout << "OBB Tree (SAH) Construction Time taken: " << time_obb_sah_construction << " ms" << std::endl
                      << std::endl;

            // Hybird tree
            start = std::chrono::high_resolution_clock::now();
            core::BVH hybrid(triangles, true, true);
            end = std::chrono::high_resolution_clock::now();
            auto time_hybrid_build = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            std::cout << "Hybrid Tree Construction Time taken: " << time_hybrid_build << " ms" << std::endl << std::endl;

            results.emplace_back("AABB BVH (No OBB)", time_aabb_construction);
            results.emplace_back("AABB BVH (Has OBB)", time_aabb_withobb_construction);
            results.emplace_back("OBB Tree (Midpoint)", time_obb_construction);
            results.emplace_back("OBB Tree (Median)", time_obb_median_construction);
            results.emplace_back("OBB Tree (SAH)", time_obb_sah_construction);
            results.emplace_back("Hybrid Tree", time_hybrid_build);

            // Save to each filecode
            saveToCSV("results/Analysis_BVH_Build_time", "construction_time_" + code + ".csv",
                      "BVH Type, Construction Time (ms)", results);
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
        for (const auto &node: nodes)
        {
            // DO not consider the child nodes
            totalSAHCost += node.obb.area() * node.count;;
        }
        return totalSAHCost;
    }

    double calculateHybridTreeSAHCost(BVH &bvh)
    {
        std::vector<Node> nodes = bvh.getNodes();
        double totalSAHCost = 0.0;

        for (const auto &node: nodes)
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

        for (const std::string &code: fileCode)
        {
            std::vector<std::pair<std::string, double>> results;
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Processing file: " << code << std::endl;
            std::string inputFile = code;

            std::vector<std::vector<Triangle>> models;
            models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Semi + inputFile, false);
            std::vector<Triangle> triangles;

            for (const auto &model: models)
            {
                triangles.insert(triangles.end(), model.begin(), model.end());
            }

            // AABB BVH without OBB
            BVH bvh(triangles, false, false);
            double sahCost_aabb = calculateAABBTreeSAHCost(bvh, false);
            std::cout << "SAH Cost of AABB (No OBB) BVH: " << sahCost_aabb << std::endl;

            // AABB BVH with OBB
            BVH bvh_obb(triangles, true, false);
            double sahCost_aabb_obb = calculateAABBTreeSAHCost(bvh_obb, true);
            std::cout << "SAH Cost of AABB (Has OBB) BVH: " << sahCost_aabb_obb << std::endl;

            // OBB tree Midpoint
            core::obb::ObbTree obbTree(triangles, false, false);
            double sahCost_obb_mid = CalculateOBBTreeSAHCost(obbTree);
            std::cout << "SAH Cost of OBB Tree (Midpoint): " << sahCost_obb_mid << std::endl;

            // OBB tree Median
            core::obb::ObbTree obbTree_median(triangles, false, false, 16, 0, true);
            double sahCost_obb_median = CalculateOBBTreeSAHCost(obbTree_median);
            std::cout << "SAH Cost of OBB Tree (Median): " << sahCost_obb_median << std::endl;

            // OBB tree SAH
            core::obb::ObbTree obbTree_sah(triangles, true, false);
            double sahCost_obb_sah = CalculateOBBTreeSAHCost(obbTree_sah);
            std::cout << "SAH Cost of OBB Tree (SAH): " << sahCost_obb_sah << std::endl;

            // Hybrid tree
            BVH bvh_hybrid(triangles, true, true);
            double sahCost_hybrid = calculateHybridTreeSAHCost(bvh_hybrid);
            std::cout << "SAH Cost of Hybrid Tree: " << sahCost_hybrid << std::endl << std::endl;

            results.emplace_back("AABB BVH (No OBB)", sahCost_aabb);
            results.emplace_back("AABB BVH (Has OBB)", sahCost_aabb_obb);
            results.emplace_back("OBB Tree (Midpoint)", sahCost_obb_mid);
            results.emplace_back("OBB Tree (Median)", sahCost_obb_median);
            results.emplace_back("OBB Tree (SAH)", sahCost_obb_sah);
            results.emplace_back("Hybrid Tree", sahCost_hybrid);

            // Save to each filecode
            saveToCSV("results/Analysis_BVH_Metrics_SAHCost", "sah_cost_" + code + ".csv",
                      "BVH Type, SAH Cost", results);
        }
    }


    TEST(FlatRace, BVH_Mtrics_stddev)
    {
        std::vector<std::string> fileCode = {"a", "b", "c", "d", "e", "f", "g"};

        for (const std::string &code: fileCode)
        {
            std::vector<std::pair<std::string, double>> results;
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Processing file: " << code << std::endl;
            std::string inputFile = code;

            std::vector<std::vector<Triangle>> models;
            models = utils::Obj::loadAllObjFilesInFolder(TEST_OBJ_FOLDER_Semi + inputFile, false);
            std::vector<Triangle> triangles;

            for (const auto &model: models)
            {
                triangles.insert(triangles.end(), model.begin(), model.end());
            }

            // AABB BVH without OBB
            BVH bvh(triangles, false, false);
            std::vector<int> leafDepths = bvh.getLeafDepths();
            double mean = std::accumulate(leafDepths.begin(), leafDepths.end(), 0.0) / leafDepths.size();
            double sq_sum = std::inner_product(leafDepths.begin(), leafDepths.end(), leafDepths.begin(), 0.0);
            double stddev = std::sqrt(sq_sum / leafDepths.size() - mean * mean);
            std::cout << "Standard Deviation of AABB (No OBB) BVH Leaf Depths: " << stddev << std::endl;

            // AABB BVH with OBB
            BVH bvh_obb(triangles, true, false);
            std::vector<int> leafDepths_obb = bvh_obb.getLeafDepths();
            double mean_obb = std::accumulate(leafDepths_obb.begin(), leafDepths_obb.end(), 0.0) / leafDepths_obb.size();
            double sq_sum_obb = std::inner_product(leafDepths_obb.begin(), leafDepths_obb.end(), leafDepths_obb.begin(),
                                                   0.0);
            double stddev_obb = std::sqrt(sq_sum_obb / leafDepths_obb.size() - mean_obb * mean_obb);
            std::cout << "Standard Deviation of AABB (Has OBB) BVH Leaf Depths: " << stddev_obb << std::endl;

            // OBB tree Midpoint
            core::obb::ObbTree obbTree(triangles, false, false);
            std::vector<int> leafDepths_obb_mid = obbTree.getLeafDepths();
            double mean_obb_mid = std::accumulate(leafDepths_obb_mid.begin(), leafDepths_obb_mid.end(), 0.0) / leafDepths_obb_mid.size();
            double sq_sum_obb_mid = std::inner_product(leafDepths_obb_mid.begin(), leafDepths_obb_mid.end(), leafDepths_obb_mid.begin(),
                                                   0.0);
            double stddev_obb_mid = std::sqrt(sq_sum_obb_mid / leafDepths_obb_mid.size() - mean_obb_mid * mean_obb_mid);
            std::cout << "Standard Deviation of OBB Tree (Midpoint) Leaf Depths: " << stddev_obb_mid << std::endl;

            // OBB tree Median
            core::obb::ObbTree obbTree_median(triangles, false, false, 16, 0, true);
            std::vector<int> leafDepths_obb_median = obbTree_median.getLeafDepths();
            double mean_obb_median = std::accumulate(leafDepths_obb_median.begin(), leafDepths_obb_median.end(), 0.0) / leafDepths_obb_median.size();

            double sq_sum_obb_median = std::inner_product(leafDepths_obb_median.begin(), leafDepths_obb_median.end(), leafDepths_obb_median.begin(),
                                                   0.0);
            double stddev_obb_median = std::sqrt(sq_sum_obb_median / leafDepths_obb_median.size() - mean_obb_median * mean_obb_median);
            std::cout << "Standard Deviation of OBB Tree (Median) Leaf Depths: " << stddev_obb_median << std::endl;

            // OBB tree SAH
            core::obb::ObbTree obbTree_sah(triangles, true, false);
            std::vector<int> leafDepths_obb_sah = obbTree_sah.getLeafDepths();
            double mean_obb_sah = std::accumulate(leafDepths_obb_sah.begin(), leafDepths_obb_sah.end(), 0.0) / leafDepths_obb_sah.size();
            double sq_sum_obb_sah = std::inner_product(leafDepths_obb_sah.begin(), leafDepths_obb_sah.end(), leafDepths_obb_sah.begin(),
                                                   0.0);
            double stddev_obb_sah = std::sqrt(sq_sum_obb_sah / leafDepths_obb_sah.size() - mean_obb_sah * mean_obb_sah);
            std::cout << "Standard Deviation of OBB Tree (SAH) Leaf Depths: " << stddev_obb_sah << std::endl;

            // Hybrid tree
            BVH bvh_hybrid(triangles, true, true);
            std::vector<int> leafDepths_hybrid = bvh_hybrid.getLeafDepths();
            double mean_hybrid = std::accumulate(leafDepths_hybrid.begin(), leafDepths_hybrid.end(), 0.0) / leafDepths_hybrid.size();
            double sq_sum_hybrid = std::inner_product(leafDepths_hybrid.begin(), leafDepths_hybrid.end(), leafDepths_hybrid.begin(),
                                                   0.0);
            double stddev_hybrid = std::sqrt(sq_sum_hybrid / leafDepths_hybrid.size() - mean_hybrid * mean_hybrid);
            std::cout << "Standard Deviation of Hybrid Tree Leaf Depths: " << stddev_hybrid << std::endl << std::endl;

            results.emplace_back("AABB BVH (No OBB)", stddev);
            results.emplace_back("AABB BVH (Has OBB)", stddev_obb);
            results.emplace_back("OBB Tree (Midpoint)", stddev_obb_mid);
            results.emplace_back("OBB Tree (Median)", stddev_obb_median);
            results.emplace_back("OBB Tree (SAH)", stddev_obb_sah);
            results.emplace_back("Hybrid Tree", stddev_hybrid);

            // Save to each filecode
            saveToCSV("results/Analysis_BVH_Metrics_StdDev", "stddev_" + code + ".csv",
                      "BVH Type, Standard Deviation", results);
        }
    }

    // Basic render time
    TEST(FlatRace, Analysis_Render_time)
    {
        std::vector<std::string> fileCode = {"a", "b", "c", "d", "e", "f", "g"};

        for (const std::string &code: fileCode)
        {
            std::vector<std::pair<std::string, double>> results;
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
            BVH bvh(triangles, false, false);

            // AABB with OBB
            BVH bvh_obb(triangles, true, false);

            // OBB Midpoint
            core::obb::ObbTree obbTree(triangles, false, false);

            // OBB Median
            core::obb::ObbTree obbTreeMedian(triangles, false, false, 16, 0, true);

            // OBB SAH
            core::obb::ObbTree obbTreeSAH(triangles, true, false);

            // Hybrid tree
            BVH bvh_hybrid(triangles, true, true);

            Frame frame(FRAME_WIDTH, FRAME_HEIGHT);

            double totalTimeAABB = 0.0;
            double totalTimeAABB_OBB = 0.0;
            double totalTimeOBB = 0.0;
            double totalTimeOBBSAH = 0.0;
            double totalTimeOBBMedian = 0.0;
            double totalTimeHybrid = 0.0;

            for (int i = 0; i < 10; ++i)
            {
                float angle = 2 * PI * i / 10;
                float cx = std::cos(angle) * 2.0f;
                float cz = std::sin(angle) * 2.0f;
                Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};

                // OBB midpoint
                auto start = std::chrono::high_resolution_clock::now();
                render_frameOBB(camera, obbTree, frame.pixels.get(), false, false);
                auto end = std::chrono::high_resolution_clock::now();
                auto time_obb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                totalTimeOBB += time_obb;

                // OBB Median
                start = std::chrono::high_resolution_clock::now();
                render_frameOBB(camera, obbTreeMedian, frame.pixels.get(), false, false);
                end = std::chrono::high_resolution_clock::now();
                auto time_obb_median = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                totalTimeOBBMedian += time_obb_median;

                // OBB SAH
                start = std::chrono::high_resolution_clock::now();
                render_frameOBB(camera, obbTreeSAH, frame.pixels.get(), false, false);
                end = std::chrono::high_resolution_clock::now();
                auto time_obb_sah = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                totalTimeOBBSAH += time_obb_sah;

                // AABB without OBB
                start = std::chrono::high_resolution_clock::now();
                render_frame(camera, bvh, frame.pixels.get(), false, false, false);
                end = std::chrono::high_resolution_clock::now();
                auto time_aabb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                totalTimeAABB += time_aabb;

                // AABB with OBB
                start = std::chrono::high_resolution_clock::now();
                render_frame(camera, bvh_obb, frame.pixels.get(), true, false, false);
                end = std::chrono::high_resolution_clock::now();
                auto time_aabb_obb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                totalTimeAABB_OBB += time_aabb_obb;

                // Hybrid tree
                start = std::chrono::high_resolution_clock::now();
                render_frameHybrid(camera, bvh_hybrid, frame.pixels.get(), false, false);
                end = std::chrono::high_resolution_clock::now();
                auto time_hybrid = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                totalTimeHybrid += time_hybrid;

            }

            std::cout << "Average Render Semiconductor AABB (No OBB) Time taken: " << totalTimeAABB / 10.0 << " ms"
                      << std::endl;
            std::cout << "Average Render Semiconductor AABB (Use OBB) Time taken: " << totalTimeAABB_OBB / 10.0 << " ms"
                      << std::endl;
            std::cout << "Average Render Semiconductor OBB (Midpoint) Time taken: " << totalTimeOBB / 10.0 << " ms"
                      << std::endl;
            std::cout << "Average Render Semiconductor OBB (SAH) Time taken: " << totalTimeOBBSAH / 10.0 << " ms"
                      << std::endl;
            std::cout << "Average Render Semiconductor OBB (Median) Time taken: " << totalTimeOBBMedian / 10.0 << " ms"
                        << std::endl;
            std::cout << "Average Render Semiconductor Hybrid Time taken: " << totalTimeHybrid / 10.0 << " ms"
                        << std::endl << std::endl;

            results.emplace_back("AABB BVH (No OBB)", totalTimeAABB / 10.0);
            results.emplace_back("AABB BVH (Has OBB)", totalTimeAABB_OBB / 10.0);
            results.emplace_back("OBB Tree (Midpoint)", totalTimeOBB / 10.0);
            results.emplace_back("OBB Tree (Median)", totalTimeOBBMedian / 10.0);
            results.emplace_back("OBB Tree (SAH)", totalTimeOBBSAH / 10.0);
            results.emplace_back("Hybrid Tree", totalTimeHybrid / 10.0);

            // Save to each filecode
            saveToCSV("results/Analysis_Render_time", "render_time_" + code + ".csv",
                      "BVH Type, Render Time", results);
        }
    }

    // Test AABB SAH Bin num
    TEST(FlatRace, AABB_SAH_Bin)
    {
        std::vector<std::string> fileCode = {"a", "b", "c", "d", "e", "f", "g"};
        std::vector<int> binNum = {5, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150};

        for (const std::string &code: fileCode)
        {
            std::vector<std::pair<std::string, double>> results;
            std::vector<std::pair<std::string, double>> results_build_time;
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

            for (int num: binNum)
            {
                // AABB SAH with obb
                auto start_build = std::chrono::high_resolution_clock::now();
                BVH bvh(triangles, true, false, num);
                auto end_build = std::chrono::high_resolution_clock::now();
                auto time_build = std::chrono::duration_cast<std::chrono::milliseconds>(end_build - start_build).count();
                results_build_time.emplace_back(std::to_string(num), time_build);
                std::cout << "AABB BVH (obb) Construction Time taken (Bin Num: " << num << "): " << time_build
                          << " ms" << std::endl;

                Frame frame(FRAME_WIDTH, FRAME_HEIGHT);
                double totalTimeAABB = 0.0;

                for (int i = 0; i < 10; ++i)
                {
                    float angle = 2 * PI * i / 10;
                    float cx = std::cos(angle) * 2.0f;
                    float cz = std::sin(angle) * 2.0f;
                    Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};

                    // AABB SAH
                    auto start = std::chrono::high_resolution_clock::now();
                    render_frame(camera, bvh, frame.pixels.get(), false, false, false);
                    auto end = std::chrono::high_resolution_clock::now();
                    auto time_aabb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                    totalTimeAABB += time_aabb;
                }
                std::cout << "Average Render Semiconductor AABB (SAH, obb) Time taken (Bin Num: " << num << "): "
                          << totalTimeAABB / 10.0 << " ms" << std::endl;

                results.emplace_back(std::to_string(num), totalTimeAABB / 10.0);
            }

            // Save to each filecode
            saveToCSV("results/Analysis_AABB_SAH_Bin", "render_time_" + code + ".csv",
                      "Bin Num, Render Time", results);
            saveToCSV("results/Analysis_AABB_SAH_Bin", "build_time_" + code + ".csv",
                        "Bin Num, Build Time", results_build_time);
        }
    }

    // Test OBB SAH Bin num
    TEST(FlatRace, OBB_SAH_Bin)
    {
        std::vector<std::string> fileCode = {"a", "b", "c", "d", "e", "f", "g"};
        std::vector<int> binNum = {5, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150};

        for (const std::string &code: fileCode)
        {
            std::vector<std::pair<std::string, double>> results;
            std::vector<std::pair<std::string, double>> results_build_time;
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

            for (int num: binNum)
            {
                // OBB SAH
                auto start_build = std::chrono::high_resolution_clock::now();
                core::obb::ObbTree obbTreeSAH(triangles, true, false, num);
                auto end_build = std::chrono::high_resolution_clock::now();
                auto time_build = std::chrono::duration_cast<std::chrono::milliseconds>(end_build - start_build).count();
                results_build_time.emplace_back(std::to_string(num), time_build);
                std::cout << "OBB Tree (SAH) Construction Time taken (Bin Num: " << num << "): " << time_build
                          << " ms" << std::endl;

                Frame frame(FRAME_WIDTH, FRAME_HEIGHT);
                double totalTimeOBBSAH = 0.0;

                for (int i = 0; i < 10; ++i)
                {
                    float angle = 2 * PI * i / 10;
                    float cx = std::cos(angle) * 2.0f;
                    float cz = std::sin(angle) * 2.0f;
                    Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};

                    // OBB SAH
                    auto start = std::chrono::high_resolution_clock::now();
                    render_frameOBB(camera, obbTreeSAH, frame.pixels.get(), false, false);
                    auto end = std::chrono::high_resolution_clock::now();
                    auto time_obb_sah = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                    totalTimeOBBSAH += time_obb_sah;
                }
                std::cout << "Average Render Semiconductor OBB (SAH) Time taken (Bin Num: " << num << "): "
                          << totalTimeOBBSAH / 10.0 << " ms" << std::endl;

                results.emplace_back(std::to_string(num), totalTimeOBBSAH / 10.0);
            }

            // Save to each filecode
            saveToCSV("results/Analysis_OBB_SAH_Bin", "render_time_" + code + ".csv",
                      "Bin Num, Render Time", results);
            saveToCSV("results/Analysis_OBB_SAH_Bin", "build_time_" + code + ".csv",
                        "Bin Num, Build Time", results_build_time);
        }
    }

    // Testing ray caching
    TEST(FlatRace, Caching_Render_Time_AABBTree)
    {
        std::vector<std::string> fileCode = {"a", "b", "c", "d", "e", "f", "g"};

        std::vector<std::pair<std::string, double>> results;
        for (const std::string &code: fileCode)
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
            BVH bvh_obb(triangles, true, false);
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
                render_frame(camera, bvh_obb, frame.pixels.get(), true, true, false);
                auto end = std::chrono::high_resolution_clock::now();
                auto time_obb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                totalTimeOBB += time_obb;
            }

            std::cout << "Average Render Semiconductor AABB Tree(With OBB, With Caching) Time taken: "
                      << totalTimeOBB / 10.0 << " ms"
                      << std::endl << std::endl;

            results.emplace_back(code, totalTimeOBB / 10.0);
        }
        saveToCSV("results/Analysis_Caching_Render_time_AABB", "caching_render_time.csv",
                  "Model, render_time", results);
    }

    TEST(FlatRace, Caching_Render_Time_OBBTree)
    {
        std::vector<std::string> fileCode = {"a", "b", "c", "d", "e", "f", "g"};

        std::vector<std::pair<std::string, double>> results;
        for (const std::string &code: fileCode)
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

                // OBB SAH
                auto start = std::chrono::high_resolution_clock::now();
                render_frameOBB(camera, obbTree, frame.pixels.get(), false, true);
                auto end = std::chrono::high_resolution_clock::now();
                auto time_obb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                totalTimeOBB += time_obb;
            }

            std::cout << "Average Render Semiconductor OBB (No Clustering, With Caching) Time taken: "
                      << totalTimeOBB / 10.0 << " ms"
                      << std::endl << std::endl;

            results.emplace_back(code, totalTimeOBB / 10.0);

        }

        saveToCSV("results/Analysis_Caching_Render_time_OBB", "caching_render_time.csv",
                  "Model, render_time", results);
    }

    TEST(FlatRace, Caching_Render_Time_Hybrid)
    {
        std::vector<std::string> fileCode = {"a", "b", "c", "d", "e", "f", "g"};

        std::vector<std::pair<std::string, double>> results;
        for (const std::string &code: fileCode)
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
            BVH bvh_hybrid(triangles, true, true);
            Frame frame(FRAME_WIDTH, FRAME_HEIGHT);

            double totalTimeHybrid = 0.0;

            for (int i = 0; i < 10; ++i)
            {
                float angle = 2 * PI * i / 10;
                float cx = std::cos(angle) * 2.0f;
                float cz = std::sin(angle) * 2.0f;
                Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};

                //caching
                auto start = std::chrono::high_resolution_clock::now();
                render_frameHybrid(camera, bvh_hybrid, frame.pixels.get(), true, false);
                auto end = std::chrono::high_resolution_clock::now();
                auto time_hybrid = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                totalTimeHybrid += time_hybrid;
            }

            std::cout << "Average Render Semiconductor Hybrid Time taken: "
                      << totalTimeHybrid / 10.0 << " ms"
                      << std::endl << std::endl;

            results.emplace_back(code, totalTimeHybrid / 10.0);

        }

        saveToCSV("results/Analysis_Caching_Render_time_Hybrid", "caching_render_time.csv",
                  "Model, render_time", results);
    }

    TEST(FlatRace, Render_KMeans_OBB_SAH)
    {
        std::vector<std::string> fileCode = {"a", "d", "g"};
        std::vector<int> k = {1, 10, 100, 150, 200, 300, 400, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 5000};

        for (const std::string &code: fileCode)
        {
            std::vector<std::pair<std::string, double>> results_build_time;
            std::vector<std::pair<std::string, double>> results_render_time;
            std::vector<std::pair<std::string, double>> results_sah_cost;

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

            for (int j = 0; j < k.size(); ++j)
            {
                auto start = std::chrono::high_resolution_clock::now();
                core::obb::ObbTree obbTree(triangles, true, true, 32, k[j]);
                auto end = std::chrono::high_resolution_clock::now();
                float time_obb_construction = std::chrono::duration_cast<std::chrono::milliseconds>(
                        end - start).count();
                std::cout << "OBB Tree: k =  " << k[j] << " Clustering, Construction Time taken: "
                          << time_obb_construction << " ms"
                          << std::endl;

                double sahCost = CalculateOBBTreeSAHCost(obbTree);
                std::cout << "SAH Cost of OBB Tree (Clustering): " << sahCost << std::endl;

                Frame frame(FRAME_WIDTH, FRAME_HEIGHT);

                double totalTimeOBB = 0.0;

                for (int i = 0; i < 10; ++i)
                {
                    float angle = 2 * PI * i / 10;
                    float cx = std::cos(angle) * 2.0f;
                    float cz = std::sin(angle) * 2.0f;
                    Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};

                    auto start = std::chrono::high_resolution_clock::now();
                    render_frameOBB(camera, obbTree, frame.pixels.get(), true, true);
                    auto end = std::chrono::high_resolution_clock::now();
                    auto time_obb = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                    totalTimeOBB += time_obb;
                }

                std::cout << "Average Render Semiconductor OBB (Clustering) k = " << k[j] << " Time taken: "
                          << totalTimeOBB / 10.0
                          << " ms"
                          << std::endl << std::endl;

                results_render_time.emplace_back(std::to_string(k[j]), totalTimeOBB / 10.0);
                results_build_time.emplace_back(std::to_string(k[j]), time_obb_construction);
                results_sah_cost.emplace_back(std::to_string(k[j]), sahCost);

                if (j != k.size() - 1)
                {
                    if (obbTree.getLeafSize() < k[j + 1]) break;
                }
            }

            // Save to each filecode
            saveToCSV("results/Analysis_Render_KMeans_OBB/render_time", "render_time_" + code + ".csv",
                      "K, render_time", results_render_time);
            saveToCSV("results/Analysis_Render_KMeans_OBB/build_time", "build_time_" + code + ".csv",
                      "K, build_time", results_build_time);
            saveToCSV("results/Analysis_Render_KMeans_OBB/sah_cost", "sah_cost_" + code + ".csv",
                        "K, sah_cost", results_sah_cost);
        }
    }

    TEST(FlatRace, Render_KMeans_AABB)
    {
        std::vector<std::string> fileCode = {"a", "d", "g"};
        std::vector<int> k = {1, 10, 100, 150, 200, 300, 400, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 5000};

        for (const std::string &code: fileCode)
        {
            std::vector<std::pair<std::string, double>> results_build_time;
            std::vector<std::pair<std::string, double>> results_render_time;
            std::vector<std::pair<std::string, double>> results_sah_cost;

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

            for (int j = 0; j < k.size(); ++j)
            {
                auto start = std::chrono::high_resolution_clock::now();
                core::BVH bvh(triangles, true, false, 100, true, k[j]);
                auto end = std::chrono::high_resolution_clock::now();
                float time_aabb_construction = std::chrono::duration_cast<std::chrono::milliseconds>(
                        end - start).count();
                std::cout << "AABB Tree: k =  " << k[j] << " Clustering, Construction Time taken: "
                          << time_aabb_construction << " ms"
                          << std::endl;

                double sahCost = calculateAABBTreeSAHCost(bvh, true);
                std::cout << "SAH Cost of AABB Tree with OBB (Clustering): " << sahCost << std::endl;

                Frame frame(FRAME_WIDTH, FRAME_HEIGHT);

                double totalTimeAABB = 0.0;

                for (int i = 0; i < 10; ++i)
                {
                    float angle = 2 * PI * i / 10;
                    float cx = std::cos(angle) * 2.0f;
                    float cz = std::sin(angle) * 2.0f;
                    Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};

                    auto start = std::chrono::high_resolution_clock::now();
                    render_frame(camera, bvh, frame.pixels.get(), true, true, true);
                    auto end = std::chrono::high_resolution_clock::now();
                    auto time_AABB = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                    totalTimeAABB += time_AABB;
                }

                std::cout << "Average Render Semiconductor AABB with OBB (Clustering) k = " << k[j] << " Time taken: "
                          << totalTimeAABB / 10.0
                          << " ms"
                          << std::endl << std::endl;

                results_render_time.emplace_back(std::to_string(k[j]), totalTimeAABB / 10.0);
                results_build_time.emplace_back(std::to_string(k[j]), time_aabb_construction);
                results_sah_cost.emplace_back(std::to_string(k[j]), sahCost);

                if (j != k.size() - 1)
                {
                    if (bvh.getOBBLeafSize() < k[j + 1]) break;
                }
            }

            // Save to each filecode
            saveToCSV("results/Analysis_Render_KMeans_AABB/render_time", "render_time_" + code + ".csv",
                      "K, render_time", results_render_time);
            saveToCSV("results/Analysis_Render_KMeans_AABB/build_time", "build_time_" + code + ".csv",
                      "K, build_time", results_build_time);
            saveToCSV("results/Analysis_Render_KMeans_AABB/sah_cost", "sah_cost_" + code + ".csv",
                      "K, sah_cost", results_sah_cost);
        }
    }

    TEST(FlatRace, Render_KMeans_Hybrid)
    {
        std::vector<std::string> fileCode = {"a", "d", "g"};
        std::vector<int> k = {1, 10, 100, 150, 200, 300, 400, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 5000};

        for (const std::string &code: fileCode)
        {
            std::vector<std::pair<std::string, double>> results_build_time;
            std::vector<std::pair<std::string, double>> results_render_time;
            std::vector<std::pair<std::string, double>> results_sah_cost;

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

            for (int j = 0; j < k.size(); ++j)
            {
                auto start = std::chrono::high_resolution_clock::now();
                core::BVH bvh(triangles, true, true, 100, true, k[j]);
                auto end = std::chrono::high_resolution_clock::now();
                float time_hybrid_construction = std::chrono::duration_cast<std::chrono::milliseconds>(
                        end - start).count();
                std::cout << "Hybrid Tree: k =  " << k[j] << " Clustering, Construction Time taken: "
                          << time_hybrid_construction << " ms"
                          << std::endl;

                double sahCost = calculateAABBTreeSAHCost(bvh, true);
                std::cout << "SAH Cost of Hybrid Tree with OBB (Clustering): " << sahCost << std::endl;

                Frame frame(FRAME_WIDTH, FRAME_HEIGHT);

                double totalTimeHybrid = 0.0;

                for (int i = 0; i < 10; ++i)
                {
                    float angle = 2 * PI * i / 10;
                    float cx = std::cos(angle) * 2.0f;
                    float cz = std::sin(angle) * 2.0f;
                    Camera camera = {{cx, 1.0f, cz}, {-cx, -1.0f, -cz}, {0.0f, 1.0f, 0.0f}, 5.0f};

                    auto start = std::chrono::high_resolution_clock::now();
                    render_frame(camera, bvh, frame.pixels.get(), true, true, true);
                    auto end = std::chrono::high_resolution_clock::now();
                    auto time_Hybrid = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                    totalTimeHybrid += time_Hybrid;
                }

                std::cout << "Average Render Semiconductor Hybrid (Clustering) k = " << k[j] << " Time taken: "
                          << totalTimeHybrid / 10.0
                          << " ms"
                          << std::endl << std::endl;

                results_render_time.emplace_back(std::to_string(k[j]), totalTimeHybrid / 10.0);
                results_build_time.emplace_back(std::to_string(k[j]), time_hybrid_construction);
                results_sah_cost.emplace_back(std::to_string(k[j]), sahCost);

                if (j != k.size() - 1)
                {
                    if (bvh.getOBBLeafSize() < k[j + 1]) break;
                }
            }

            // Save to each filecode
            saveToCSV("results/Analysis_Render_KMeans_Hybrid/render_time", "render_time_" + code + ".csv",
                      "K, render_time", results_render_time);
            saveToCSV("results/Analysis_Render_KMeans_Hybrid/build_time", "build_time_" + code + ".csv",
                      "K, build_time", results_build_time);
            saveToCSV("results/Analysis_Render_KMeans_Hybrid/sah_cost", "sah_cost_" + code + ".csv",
                      "K, sah_cost", results_sah_cost);
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