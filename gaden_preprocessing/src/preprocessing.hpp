#pragma once
#include "Triangle.hpp"
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
    outlet = 2
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
    void generateOutput();
    void processWind();

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr jobDone_pub;

private:
    std::vector<cell_state> env;
    gaden::Vector3i dimensions; //number of cells along each axis

    // dimensions of the enviroment [m]
    gaden::Vector3 env_min;
    gaden::Vector3 env_max;
    // length of the sides of the cell [m]
    float cell_size;


    bool compare_cell(gaden::Vector3i pos, cell_state value);
    std::array<gaden::Vector3, 9> cubePoints(const gaden::Vector3& query_point);
    bool pointInTriangle(const gaden::Vector3& query_point, Triangle& triangle);

    // STL
    bool isASCII(const std::string& filename);
    void findDimensions(const std::string& filename);
    void parse(const std::string& filename, cell_state value_to_write);
    void occupy(std::vector<Triangle>& triangles, const std::vector<gaden::Vector3>& normals, cell_state value_to_write);

    //wind
    void openFoam_to_gaden(const std::string& filename);

    //output
    void changeStageWorldFile(const std::string& filename);
    void printOccupancyMap(std::string_view filename, bool block_outlets);
    void printOccupancyYaml(std::string_view outputFolder);
    void printBasicSimYaml(std::string_view outputFolder);
    void printGadenEnvFile(std::string_view filename);
    void printWindFiles(const std::vector<gaden::Vector3>& wind, std::string_view filename);


    size_t indexFrom3D(int x, int y, int z)
    {
        return x + y * dimensions.x + z * dimensions.x * dimensions.y;
    }

    size_t indexFrom3D(gaden::Vector3i vec)
    {
        return indexFrom3D(vec.x, vec.y, vec.z);
    }

};

namespace Utils
{
    inline bool approx(float x, float y)
    {
        return std::abs(x - y) < 1e-3;
    }

    inline bool isParallel(const gaden::Vector3& vec)
    {
        return (approx(vec.y, 0) && approx(vec.z, 0)) || (approx(vec.x, 0) && approx(vec.z, 0)) || (approx(vec.x, 0) && approx(vec.y, 0));
    }
} // namespace Utils