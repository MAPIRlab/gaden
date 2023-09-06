#pragma once
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <std_msgs/msg/bool.hpp>
#include <vector>

enum cell_state
{
	non_initialized = 0,
	empty = 1,
	occupied = 2,
	outlet = 3,
	edge = 4
};

struct Triangle
{
	tf2::Vector3 p1;
	tf2::Vector3 p2;
	tf2::Vector3 p3;
	Triangle() {}
	Triangle(tf2::Vector3 p1, tf2::Vector3 p2, tf2::Vector3 p3)
	{
		this->p1 = p1;
		this->p2 = p2;
		this->p3 = p3;
	}
	tf2::Vector3& operator[](int i)
	{
		if (i == 0)
			return p1;
		else if (i == 1)
			return p2;
		else if (i == 2)
			return p3;
		else
		{
			std::cout << "Indexing error when accessing the tf2::Vector3s in triangle! Index must be >= 2";
			return p1;
		}
	}
};

class Gaden_preprocessing : public rclcpp::Node
{
public:
	Gaden_preprocessing() : rclcpp::Node("Gaden_Preprocessing")
	{
		cell_size = declare_parameter<float>("cell_size", 1); // size of the cells
		roundFactor = 100.0 / cell_size;
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
	std::vector<std::vector<std::vector<int>>> env;

	// dimensions of the enviroment [m]
	float env_min_x;
	float env_min_y;
	float env_min_z;
	float env_max_x;
	float env_max_y;
	float env_max_z;
	// length of the sides of the cell [m]
	float cell_size;
	float roundFactor; // for rounding numbers to avoid certain precision problems. Depends on the cell size

	bool isASCII(const std::string& filename);

	bool compare_cell(int x, int y, int z, cell_state value);
	void changeWorldFile(const std::string& filename);
	void printMap(std::string filename, int scale, bool block_outlets);
	void printEnv(std::string filename, int scale);
	void printWind(const std::vector<double>& U,
		const std::vector<double>& V,
		const std::vector<double>& W, std::string filename);

	void printYaml(std::string output);
	std::array<tf2::Vector3, 9> cubePoints(const tf2::Vector3& query_point);
	bool pointInTriangle(const tf2::Vector3& query_point,
		const tf2::Vector3& triangle_vertex_0,
		const tf2::Vector3& triangle_vertex_1,
		const tf2::Vector3& triangle_vertex_2);

	void occupy(std::vector<Triangle>& triangles,
		const std::vector<tf2::Vector3>& normals,
		cell_state value_to_write);

	void parse(const std::string& filename, cell_state value_to_write);
	void findDimensions(const std::string& filename);
	void openFoam_to_gaden(const std::string& filename);

	int indexFrom3D(int x, int y, int z)
	{
		return x + y * env[0].size() + z * env[0].size() * env.size();
	}
};

namespace Utils
{
	float min_val(float x, float y, float z)
	{

		float min = x;
		if (y < min)
			min = y;
		if (z < min)
			min = z;

		return min;
	}
	float max_val(float x, float y, float z)
	{

		float max = x;
		if (y > max)
			max = y;
		if (z > max)
			max = z;

		return max;
	}
	bool eq(float x, float y)
	{
		return std::abs(x - y) < 0.01;
	}

	bool isParallel(const tf2::Vector3& vec)
	{
		return (eq(vec.y(), 0) && eq(vec.z(), 0)) ||
			(eq(vec.x(), 0) && eq(vec.z(), 0)) ||
			(eq(vec.x(), 0) && eq(vec.y(), 0));
	}
}