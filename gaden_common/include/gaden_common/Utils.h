#pragma once
#include <rclcpp/rclcpp.hpp>

template <typename T> T getParam(rclcpp::Node::SharedPtr node, const std::string& name, T defaultValue)
{
    if (node->has_parameter(name))
        return node->get_parameter_or<T>(name, defaultValue);
    else
        return node->declare_parameter<T>(name, defaultValue);
}