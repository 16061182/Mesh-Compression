#pragma once

#include <core/core.h>

class Compressor {
public:
	Compressor();
	~Compressor();

	// 初始化
	void init(const std::vector<Eigen::Vector3f>* in_vertices, const std::vector<std::vector<int>>* in_faces,
		const std::vector<Eigen::Vector3f>* in_normals, int _N_bins, int _patch_size_limit, float _patch_normal_tolerance, int _float_precision);
	// 执行算法并保存编码文件
	void compress_and_save(int _atoms, const std::string& save_path);
	// 根据硬patch划分写颜色数据
	void generate_patch_color(std::vector<float>* color_data);
	// 根据坐标和法线，为seed生成局部坐标系的transform
	static Eigen::Matrix4f generate_transform(const Eigen::Vector3f& cord, const Eigen::Vector3f& in_normal);
	// 记录patch相关信息
	void write_patch_info(const std::vector<std::vector<int>>*& _patch_faces, const std::vector<int>*& _vertex_to_patch, 
		const std::vector<int>*& _patch_size, int& _feature_len, int& _atoms);


private:
	// 基本参数
	int N_bins = 10; // grid划分精度，grid数量为N_bins * N_bins
	int patch_size_limit = 22;
	float patch_normal_tolerance = 90.0f;
	int float_precision = 4; // 序列化时保存小数点后几位，可以根据原始obj文件确定
	int patch_num; // patch数量
	
	// 原始数据
	const std::vector<Eigen::Vector3f>* origin_vertices; // 顶点坐标
	const std::vector<Eigen::Vector3f>* origin_normals; // 法线方向
	const std::vector<std::vector<int>>* origin_faces; // 三角形面

	// 顶点和边
	std::unordered_map<int, std::unordered_map<int, std::vector<float>>> edge_parameter; // 边的参数(0-距离，1-曲率)
	std::vector<float> vertex_curvature; // 顶点的曲率
	std::vector<int> vertex_to_patch;
	std::vector<int> vertex_to_grid;
	
	// patch相关信息
	std::vector<std::vector<int>> patch_vertices; // patch包含的顶点
	std::vector<Eigen::MatrixXf> patch_featuress; // 特征
	std::vector<Eigen::MatrixXf> patch_dictionaries; // 字典
	std::vector<Eigen::MatrixXf> patch_codes; // 编码
	std::vector<int> patch_atoms; // 算子数
	std::vector<std::vector<int>> patch_masks; // patch网格的掩码，位置为0表示该位置对应网格中不包含顶点，该网格的高度值无实际意义
	std::vector<float> patch_grid_span; // patch网格的尺寸
	std::vector<Eigen::Vector2f> patch_seed_bias; // 采样网格的位移
	std::vector<std::set<std::vector<int>>> patch_faces; // patch包含的面，每个顶点用"patch号/grid号"表示
	std::vector<std::vector<int>> patch_origin_faces; // 调试用变量，记录patch所包含的面号，每个面用origin_faces里的下标表示
	std::vector<int> patch_size; // 调试用变量，记录patch所包含的顶点数


	// 其他
	std::vector<std::set<std::vector<int>>> bi_crackfaces; // 缝隙面，有两个顶点属于相同patch
	std::set<std::vector<std::vector<int>>> tri_crackfaces; // 缝隙面，三个顶点均属于不同patch

	// 划分patches
	void generate_patches();
	// 进行重采样，返回patch特征(高度值数组)
	void resample(); // 直角坐标采样
	// 基于svd分解对特征进行编码
	static void coding(const Eigen::MatrixXf& feature, Eigen::MatrixXf& dictionary, Eigen::MatrixXf& code, int _atoms);
	// 记录连接性信息
	void record_connection();
	// 序列化
	void serialize(std::string save_path);
	// 生成边参数
	void generate_edge_parameter();
	// 用于检查中间变量的内部函数
	void check(int part);
};
