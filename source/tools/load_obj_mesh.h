#pragma once

#include <tiny_obj_loader/tiny_obj_loader_v2.h>
#include <core/core.h>


class ObjLoader {
public:
	ObjLoader();
	~ObjLoader();
	std::vector<Eigen::Vector3f>* vertices; // 所有顶点
	std::vector<std::vector<int>>* faces; // 所有面
	std::vector<Eigen::Vector3f>* normals; // 所有法线，需要初始化 
	std::vector<float>* vertex_data; // 传入shader的坐标数组
	std::vector<float>* color_data; // 传入shader的颜色数组
	int float_precision = 4; // #VA_TAG 读文件的时候记录浮点精度

	float average_edge_len;
	float scale_x; // x方向上的坐标范围
	float scale_y;
	float scale_z;

	void init(std::vector<Eigen::Vector3f>* _vertices, std::vector<std::vector<int>>* _faces, std::vector<Eigen::Vector3f>* _normals, 
		std::vector<float>* _vertex_data, std::vector<float>* _color_data);
	bool load_obj_mesh(const std::string& obj_file_path);
	void print_detail();
	void rewrite_origin_mesh(std::string save_path);

private:
	void calculate_scale();
};