#pragma once

#include <core/core.h>

class Parser {
public:
	Parser();
	~Parser();
	
	std::vector<Eigen::Vector3f>* vertices; // 所有顶点
	std::vector<std::vector<int>>* faces; // 所有面
	std::vector<float>* vertex_data; // 传入shader的坐标数组
	std::vector<float>* color_data; // 传入shader的颜色数组

	// 初始化
	void init(std::vector<Eigen::Vector3f>* _vertices, std::vector<std::vector<int>>* _faces, std::vector<float>* _vertex_data, std::vector<float>* _color_data);
	// 读取压缩文件并还原mesh
	void parse(std::string load_path);
	// 记录patch相关信息
	void write_patch_info(const std::vector<std::vector<int>>*& _patch_faces, const std::vector<int>*& _vertex_to_patch, 
		const std::vector<int>*& _patch_size, int& _feature_len, int& _atoms);

private:
	int N_bins;
	int patch_num;
	int atoms;
	std::vector<std::vector<std::vector<int>>> faces_on_grid;
	std::map<std::vector<int>, int> grid_to_vertex;
	std::vector<std::vector<int>> patch_faces; // 记录patch所包含的面号，主要用于调试
	std::vector<int> vertex_to_patch; // 记录顶点号到patch号的映射，主要用于调试
	std::vector<int> patch_size; // 记录patch所包含的顶点数，主要用于调试

	// 把patch号/grid号映射到顶点号，用于还原面数据
	void map_grid_to_vertex(int patch, int grid, const Eigen::Vector3f& cord);
};