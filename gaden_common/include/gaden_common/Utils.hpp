#pragma once
#include "gaden/core/Logging.hpp"
#include <filesystem>
#include <fmt/format.h>
#include <rclcpp/rclcpp.hpp>

namespace GadenUtils
{
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
} // namespace GadenUtils
