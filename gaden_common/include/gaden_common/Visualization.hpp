#pragma once
#include "Utils.hpp"
#include "gaden/Simulation.hpp"
#include <rclcpp/node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace GadenUtils
{
    // creates and maintains its own (static) list of publishers
    inline visualization_msgs::msg::Marker MarkerSourcePosition(rclcpp::Node* node, gaden::Simulation const& sim)
    {
        visualization_msgs::msg::Marker sourceMarker;
        {
            sourceMarker.header.frame_id = "map";
            sourceMarker.header.stamp = node->now();
            sourceMarker.ns = "Gas_Dispersion";
            sourceMarker.action = visualization_msgs::msg::Marker::ADD;
            sourceMarker.type = visualization_msgs::msg::Marker::CUBE;
            sourceMarker.scale.x = 0.15;
            sourceMarker.scale.y = 0.15;
            sourceMarker.scale.z = 0.15;

            sourceMarker.pose.position.x = sim.simulationMetadata.sourcePosition.x;
            sourceMarker.pose.position.y = sim.simulationMetadata.sourcePosition.y;
            sourceMarker.pose.position.z = sim.simulationMetadata.sourcePosition.z;

            sourceMarker.color = GadenUtils::toRosColor(sim.gasDisplayColor);
        }
        return sourceMarker;
    }
} // namespace GadenUtils