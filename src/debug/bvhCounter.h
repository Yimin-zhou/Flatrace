#pragma once

#include <atomic>

class BVHNodeCounter {
public:
    static std::atomic<int> counter;
public:
    static void increment() { counter++; }

    static int getCount() { return counter.load();}

    static void reset() { counter.store(0); }
};

