#pragma once
#include <filesystem>
#include <rclcpp/rclcpp.hpp>

template <typename T> T getParam(rclcpp::Node::SharedPtr node, const std::string& name, T defaultValue)
{
    if (node->has_parameter(name))
        return node->get_parameter_or<T>(name, defaultValue);
    else
        return node->declare_parameter<T>(name, defaultValue);
}

inline std::vector<std::filesystem::path> AsPaths(const std::vector<std::string>& strs)
{
    std::vector<std::filesystem::path> paths;
    paths.reserve(strs.size());
    for (const auto& str : strs)
        paths.emplace_back(str);
    return paths;
}