#pragma once
#include "Utils.hpp"
#include "gaden/Simulation.hpp"
#include "gaden/datatypes/sources/BoxSource.hpp"
#include "gaden/datatypes/sources/CylinderSource.hpp"
#include "gaden/datatypes/sources/LineSource.hpp"
#include "gaden/datatypes/sources/SphereSource.hpp"
#include "gaden/internal/Pointers.hpp"
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

            sourceMarker.pose.position.x = sim.simulationMetadata.source->sourcePosition.x;
            sourceMarker.pose.position.y = sim.simulationMetadata.source->sourcePosition.y;
            sourceMarker.pose.position.z = sim.simulationMetadata.source->sourcePosition.z;

            sourceMarker.color = GadenUtils::toRosColor(sim.gasDisplayColor);
            sourceMarker.color.a = 0.6;

            std::string sourceType = sim.simulationMetadata.source->Type();
            if (sourceType == "point")
            {
                sourceMarker.type = visualization_msgs::msg::Marker::CUBE;
                sourceMarker.scale.x = 0.1;
                sourceMarker.scale.y = 0.1;
                sourceMarker.scale.z = 0.1;
            }
            else if (sourceType == "box")
            {
                sourceMarker.type = visualization_msgs::msg::Marker::CUBE;
                sourceMarker.scale.x = As<gaden::BoxSource>(sim.simulationMetadata.source)->size.x;
                sourceMarker.scale.y = As<gaden::BoxSource>(sim.simulationMetadata.source)->size.y;
                sourceMarker.scale.z = As<gaden::BoxSource>(sim.simulationMetadata.source)->size.z;
            }
            else if (sourceType == "sphere")
            {
                sourceMarker.type = visualization_msgs::msg::Marker::SPHERE;
                float r = As<gaden::SphereSource>(sim.simulationMetadata.source)->radius;
                sourceMarker.scale.x = r * 2.;
                sourceMarker.scale.y = r * 2.;
                sourceMarker.scale.z = r * 2.;
            }
            else if (sourceType == "line")
            {
                sourceMarker.type = visualization_msgs::msg::Marker::LINE_LIST;

                gaden::Vector3 start = sim.simulationMetadata.source->sourcePosition;
                sourceMarker.points.push_back(geometry_msgs::msg::Point() //
                                                  .set__x(start.x)        //
                                                  .set__y(start.y)        //
                                                  .set__z(start.z));

                gaden::Vector3 end = As<gaden::LineSource>(sim.simulationMetadata.source)->lineEnd;
                sourceMarker.points.push_back(geometry_msgs::msg::Point() //
                                                  .set__x(end.x)        //
                                                  .set__y(end.y)        //
                                                  .set__z(end.z));

                sourceMarker.pose = geometry_msgs::msg::Pose();
                sourceMarker.scale.x = .05f;
                sourceMarker.colors.push_back(sourceMarker.color); // necessary because the only line marker is a list :(
            }
            else if (sourceType == "cylinder")
            {
                sourceMarker.type = visualization_msgs::msg::Marker::CYLINDER;
                float r = As<gaden::CylinderSource>(sim.simulationMetadata.source)->radius;
                float h = As<gaden::CylinderSource>(sim.simulationMetadata.source)->height;
                sourceMarker.scale.x = r * 2.;
                sourceMarker.scale.y = r * 2.;
                sourceMarker.scale.z = h;
            }
        }
        return sourceMarker;
    }
} // namespace GadenUtils