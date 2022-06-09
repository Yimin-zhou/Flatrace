# flatrace

Fast CPU flat-shading ray trace experiment.

Who doesn't like real-time RTX GPU based path tracing with global illumination and everything? Everyone does.
But what about plain-old CPU-based raytracing of flat-shaded polygons, and nothing else? No shadows, no 
reflections, just oldskool polygons. Computers are really, really fast, but how fast exactly? Let's find out.

#### Disclaimer
The code in this repository is all really 'get it done' quality code, for experimentation only. Not indicative
of any coding standards I usually try to keep myself to :)

### Baseline performance

![bunny](images/bunny.png "Hello bunny")

It's our old friend Stanford Bunny. Rendering the 1024x768 image above using the most naive, non-optimized 
version of our BVH-based raytracer takes ~200ms on a single core of an M1 MacBook Pro. This translates to ~4 
million rays per second. By no means slow, but not nearly fast enough yet, we can do much better.

First batch of low-hanging fruit ideas:

  * Replace recursive BVH traversal by a stack + loop so it can be fully inlined
  * Render in 4x4 tiles to improve spatial locality, to reduce cache misses
  * Choose a better starting point for BVH traversal, and traverse closest leaf nodes first
  * Tune BVH leaf node size, initial guess is that BVH depth now outweighs reducing the number of triangle intersections
  * Optimize bounding box slab test

After that, things get more complicated, but the following thigs could all yield huge performance gains:

  * Use surface area or other heuristic for BVH building
  * Use SIMD for 4-way (M1, NEON) or even 8-way (x86, AVX2) 'ray packet' intersections
  * Checkerboard rendering with temporal interpolation (thi is cheating of course, but why not)
  * Multithreaded rendering (trivial to implement, but let's save the best for last)


