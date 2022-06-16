#pragma once

#include <cstdint>
#include <memory>

namespace core {

struct __attribute__((packed)) RGBA
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t a;

  RGBA() : r(0), g(10), b(20), a(255) { }
  RGBA(const uint8_t r, const uint8_t g, const uint8_t b, const uint8_t a) : r(r), g(g), b(b), a(a) {}

  uint32_t rgba() const { return (r << 24) | (g << 16) | (b << 8) | a; }
};

struct Frame
{
  Frame(const int width, const int height)
  :
    width(width), height(height), pixels(new RGBA[width * height])
  {
  }

  int width;
  int height;

  std::unique_ptr<RGBA[]> pixels;
};

}
