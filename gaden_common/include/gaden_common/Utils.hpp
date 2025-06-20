#pragma once
#include "gaden/core/Logging.hpp"
#include <filesystem>
#include <fmt/format.h>
#include <rclcpp/rclcpp.hpp>
#include <gaden/datatypes/Color.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace GadenUtils
{
    inline void OldProjectWarning()
    {
        GADEN_WARN("\n"
                   "--- Deprecation notice ---\n"
                   "You are using ROS parameters to specify the Gaden configuration. This is now considered deprecated (though, for now, still valid).\n"
                   "You should update your configuration to use a gaden project (see test_env for examples).\n"
                   "You can easily configure a gaden project with 'ros2 run gaden_common gaden_gui\n"
                   "----");
    }

    template <typename T> T getParam(rclcpp::Node::SharedPtr node, const std::string& name, T defaultValue)
    {
        if (node->has_parameter(name))
            return node->get_parameter_or<T>(name, defaultValue);
        else
            return node->declare_parameter<T>(name, defaultValue);
    }

    // takes fmt formatting that expects [common part of file path] [index] as dynamic parameters
    inline std::vector<std::filesystem::path> GetWindFiles(std::function<std::string(std::string const&, size_t)> fmt, std::string const& commonPath)
    {
        std::string filename = fmt(commonPath, 0);
        if (!std::filesystem::exists(filename))
        {
            GADEN_WARN("File '{}' does not exist", filename.c_str());
            return {};
        }

        size_t idx = 0;
        std::vector<std::filesystem::path> paths;
        for (; std::filesystem::exists(filename); filename = fmt(commonPath, idx))
        {
            paths.emplace_back(filename);
            idx++;
        }
        return paths;
    }

    inline std_msgs::msg::ColorRGBA toRosColor(gaden::Color color)
    {
        std_msgs::msg::ColorRGBA col;
        col.r = color.r;
        col.g = color.g;
        col.b = color.b;
        col.a = color.a;
        return col;
    }
} // namespace GadenUtils
