#include "ppm.h"
#include "fmt/format-inl.h"
#include <fstream>

namespace utils::Ppm
{
    void write(const std::string &filename, const core::Frame &frame)
    {
        // Open the file for writing
        std::ofstream out(filename, std::ios::binary);
        if (!out.is_open())
        {
            throw std::runtime_error(fmt::format("Failed to open PPM file '{0}' for writing", filename));
        }

        // Write PPM header
        out << "P6\n";
        out << fmt::format("{0} {1}\n", frame.width, frame.height);
        out << "255\n";

        // Write pixel data
        std::for_each_n(frame.pixels.get(), frame.width * frame.height, [&out](const core::RGBA &rgba)
        {
            out.write(reinterpret_cast<const char *>(&rgba.r), 1);
            out.write(reinterpret_cast<const char *>(&rgba.g), 1);
            out.write(reinterpret_cast<const char *>(&rgba.b), 1);
        });

        // Close the file
        out.close();
    }
}
