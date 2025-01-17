#pragma once
#include "gaden_common/Logging.h"
#include "gaden_common/Vector3.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string_view>
#include <tf2/LinearMath/Vector3.h>
#include <vector>

enum cell_state
{
    non_initialized = 9,
    empty = 0,
    occupied = 1,
    outlet = 2,
    edge = 4
};

struct Triangle
{
    gaden::Vector3 p1;
    gaden::Vector3 p2;
    gaden::Vector3 p3;
    Triangle()
    {}
    Triangle(const gaden::Vector3& p1, const gaden::Vector3& p2, const gaden::Vector3& p3)
    {
        this->p1 = p1;
        this->p2 = p2;
        this->p3 = p3;
    }
    gaden::Vector3& operator[](int i)
    {
        if (i == 0)
            return p1;
        else if (i == 1)
            return p2;
        else if (i == 2)
            return p3;
        else
        {
            GADEN_ERROR("Indexing error when accessing the gaden::Vector3s in triangle! Index must be >= 2");
            return p1;
        }
    }
};

class Gaden_preprocessing : public rclcpp::Node
{
public:
    Gaden_preprocessing()
        : rclcpp::Node("Gaden_Preprocessing")
    {
        cell_size = declare_parameter<float>("cell_size", 1); // size of the cells
        jobDone_pub = create_publisher<std_msgs::msg::Bool>("preprocessing_done", 10);
    }

    void parseMainModels();
    void parseOutletModels();
    void fill();
    void clean();
    void generateOutput();
    void processWind();

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr jobDone_pub;

private:
    std::vector<cell_state> env;
    gaden::Vector3i dimensions;

    // dimensions of the enviroment [m]
    gaden::Vector3 env_min;
    gaden::Vector3 env_max;
    // length of the sides of the cell [m]
    float cell_size;

    bool isASCII(const std::string& filename);

    bool compare_cell(gaden::Vector3i pos, cell_state value);
    void changeStageWorldFile(const std::string& filename);
    void printOccupancyMap(std::string_view filename, bool block_outlets);
    void printOccupancyYaml(std::string_view outputFolder);
    void printBasicSimYaml(std::string_view outputFolder);
    void printGadenEnvFile(std::string_view filename);
    void printWindFiles(const std::vector<gaden::Vector3>& wind, std::string_view filename);

    std::array<gaden::Vector3, 9> cubePoints(const gaden::Vector3& query_point);
    bool pointInTriangle(const gaden::Vector3& query_point, const gaden::Vector3& triangle_vertex_0, const gaden::Vector3& triangle_vertex_1,
                         const gaden::Vector3& triangle_vertex_2);

    void occupy(std::vector<Triangle>& triangles, const std::vector<gaden::Vector3>& normals, cell_state value_to_write);

    void parse(const std::string& filename, cell_state value_to_write);
    void findDimensions(const std::string& filename);
    void openFoam_to_gaden(const std::string& filename);

    size_t indexFrom3D(int x, int y, int z)
    {
        return y + x * dimensions.y + z * dimensions.x * dimensions.y;
    }

    size_t indexFrom3D(gaden::Vector3i vec)
    {
        return indexFrom3D(vec.x, vec.y, vec.z);
    }

};

namespace Utils
{
    inline bool eq(float x, float y)
    {
        return std::abs(x - y) < 0.01;
    }

    inline bool isParallel(const gaden::Vector3& vec)
    {
        return (eq(vec.y, 0) && eq(vec.z, 0)) || (eq(vec.x, 0) && eq(vec.z, 0)) || (eq(vec.x, 0) && eq(vec.y, 0));
    }
} // namespace Utils