#pragma once

#include "src/core/bvh.h"

namespace debug
{
    // Show memory usage
    // From:http://NadeauSoftware.com/
#if defined(_WIN32)
    #include <windows.h>
    #include <psapi.h>

#elif defined(__unix__) || defined(__unix) || defined(unix) || (defined(__APPLE__) && defined(__MACH__))
    #include <unistd.h>
    #include <sys/resource.h>

#if defined(__APPLE__) && defined(__MACH__)
    #include <mach/mach.h>

#elif (defined(_AIX) || defined(__TOS__AIX__)) || (defined(__sun__) || defined(__sun) || defined(sun) && (defined(__SVR4) || defined(__svr4__)))
    #include <fcntl.h>
    #include <procfs.h>

#elif defined(__linux__) || defined(__linux) || defined(linux) || defined(__gnu_linux__)
    #include <stdio.h>
#endif

#else
    #error "Cannot define getPeakRSS( ) or getCurrentRSS( ) for an unknown OS."
#endif

/**
 * Returns the current resident set size (physical memory use) measured
 * in bytes, or zero if the value cannot be determined on this OS.
 */
    size_t getCurrentRSS( )
    {
#if defined(_WIN32)
        // Windows
        PROCESS_MEMORY_COUNTERS info;
        GetProcessMemoryInfo( GetCurrentProcess( ), &info, sizeof(info) );
        return (size_t)info.WorkingSetSize;

#elif defined(__APPLE__) && defined(__MACH__)
        // OXS
        struct mach_task_basic_info info;
        mach_msg_type_number_t infoCount = MACH_TASK_BASIC_INFO_COUNT;
        if ( task_info( mach_task_self( ), MACH_TASK_BASIC_INFO,
            (task_info_t)&info, &infoCount ) != KERN_SUCCESS )
            return (size_t)0L;      /* Can't access? */
        return (size_t)info.resident_size;

#elif defined(__linux__) || defined(__linux) || defined(linux) || defined(__gnu_linux__)
        // Linux
        long rss = 0L;
        FILE* fp = NULL;
        if ( (fp = fopen( "/proc/self/statm", "r" )) == NULL )
            return (size_t)0L;      /* Can't open? */
        if ( fscanf( fp, "%*s%ld", &rss ) != 1 )
        {
            fclose( fp );
            return (size_t)0L;      /* Can't read? */
        }
        fclose( fp );
        return (size_t)rss * (size_t)sysconf( _SC_PAGESIZE);

#else
        return (size_t)0L;          /* Unsupported. */
#endif
    }

    // Show the BVH tree in the ImGui window
    void renderBVHtree(const core::Node *node, const std::vector<core::Node> &nodes, int depth = 0) {
        if (!node) return;

        ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_SpanAvailWidth;
        if (node->isLeaf()) flags |= ImGuiTreeNodeFlags_Leaf;

        bool nodeOpen = ImGui::TreeNodeEx((void *) (intptr_t) node, flags, "Node Depth %d", depth);

        if (ImGui::IsItemClicked()) {
            // Handle node click, if necessary
        }

        if (nodeOpen) {
            ImGui::Text("Bounding Box: Min(%f, %f, %f) Max(%f, %f, %f)",
                        node->bbox.min.x, node->bbox.min.y, node->bbox.min.z,
                        node->bbox.max.x, node->bbox.max.y, node->bbox.max.z);

            if (node->isLeaf()) {
                ImGui::Text("Leaf Node: Triangles from %d to %d", node->leftFrom, node->leftFrom + node->count);
            } else {
                renderBVHtree(&nodes[node->leftFrom], nodes, depth + 1);
                renderBVHtree(&nodes[node->leftFrom + 1], nodes, depth + 1);
            }
            ImGui::TreePop();
        }
    }

}