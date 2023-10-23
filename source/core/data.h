#pragma once

#include <core/core.h>

struct Data {
	std::vector<Eigen::Vector3f> vertices; // 所有顶点
	std::vector<std::vector<int>> faces; // 所有面
	std::vector<Eigen::Vector3f> normals; // 所有法线，需要初始化 
	std::vector<float> vertex_data; // 传入shader的坐标数组
	std::vector<float> color_data; // 传入shader的颜色数组

	const std::vector<std::vector<int>>* patch_faces; // 记录patch所包含的面号，主要用于调试
	const std::vector<int>* vertex_to_patch; // 记录顶点号到patch号的映射，主要用于调试
	const std::vector<int>* patch_size; // 记录patch所包含的顶点数，主要用于调试

	int feature_len; // 特征长度
	int atoms; // 算子数
};

struct Config {
	Config(std::string json_file);

	int atoms;
	int N_bins;
	int patch_size_limit;
	float patch_normal_tolerance;
	int float_precision;
};