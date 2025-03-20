#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "Vector3.h"

namespace gaden
{
    static tf2::Vector3 toTF(const Vector3& v)
    {
        return tf2::Vector3(v.x, v.y, v.z);
    }

    static tf2::Vector3 toTF(const Vector3i& v)
    {
        return tf2::Vector3(v.x, v.y, v.z);
    }

    static Vector3 fromTF(const tf2::Vector3& v)
    {
        return Vector3(v.x(), v.y(), v.z());
    }

    static geometry_msgs::msg::Vector3 toGeoMSg(const Vector3& v)
    {
        geometry_msgs::msg::Vector3 g;
        g.x = v.x;
        g.y = v.y;
        g.z = v.z;
        return g;
    }

    static Vector3 fromGeoMSg(const geometry_msgs::msg::Vector3& v)
    {
        return Vector3(v.x, v.y, v.z);
    }

    static geometry_msgs::msg::Point toPoint(const Vector3& v)
    {
        geometry_msgs::msg::Point g;
        g.x = v.x;
        g.y = v.y;
        g.z = v.z;
        return g;
    }

    static Vector3 fromPoint(const geometry_msgs::msg::Point& v)
    {
        return Vector3(v.x, v.y, v.z);
    }

    static geometry_msgs::msg::Point geoMsgToPoint(const geometry_msgs::msg::Vector3& vec)
    {
        geometry_msgs::msg::Point p;
        p.x = vec.x;
        p.y = vec.y;
        p.z = vec.z;

        return p;
    }

} // namespace gaden