#include "ppm.h"

#include "fmt/format-inl.h"

#include <fstream>

namespace utils::Ppm
{

    void write(const std::string &filename, const core::Frame &frame)
    {
        // Create a PPM file and write the frame to it if it doesn't exist


        std::ofstream out(filename, std::ios::binary);

        if (!out.is_open())
        {
            throw std::runtime_error(fmt::format("Failed to open PPM file '{0}' for writing", filename));
        }

        out << "P6\n";
        out << fmt::format("{0} {1}\n", frame.width, frame.height);
        out << "255\n";

        std::for_each_n(frame.pixels.get(), frame.width * frame.height, [&out](const core::RGBA &rgba)
        {
            out.write(reinterpret_cast<const char *>(&rgba.r), 1);
            out.write(reinterpret_cast<const char *>(&rgba.g), 1);
            out.write(reinterpret_cast<const char *>(&rgba.b), 1);
        });

        out.close();
    }

}
