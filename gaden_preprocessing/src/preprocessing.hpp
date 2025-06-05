#pragma once
#include "gaden/Environment.hpp"
#include "gaden/internal/WindSequence.hpp"
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <gaden/core/Vectors.hpp>
#include <gaden/EnvironmentConfigMetadata.hpp>

class Gaden_preprocessing : public rclcpp::Node
{
public:
    Gaden_preprocessing()
        : rclcpp::Node("Gaden_Preprocessing")
    {
        jobDone_pub = create_publisher<std_msgs::msg::Bool>("preprocessing_done", 10);
    }

    void Run();

private:
    std::vector<std::filesystem::path> GetModels(const std::string& param_name);
    gaden::WindSequence GetWindSequenceROSParams(const gaden::Environment& env);
    gaden::WindSequence GetWindSequenceProject(const gaden::Environment& env);

private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr jobDone_pub;
    std::optional<gaden::EnvironmentConfigMetadata> gadenProject;
};
