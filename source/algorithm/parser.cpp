#include "parser.h"

#include <algorithm/compressor.h>
#include <fstream>
#include <iostream>

Parser::Parser() {
}

Parser::~Parser() {
}

void Parser::map_grid_to_vertex(int patch, int grid, const Eigen::Vector3f& cord) {
	grid_to_vertex[{patch, grid}] = vertices->size();
	vertices->push_back(cord);
	vertex_to_patch.push_back(patch);
}

void Parser::parse(std::string load_path) {
	std::ifstream infile(load_path);
	if (!infile.is_open()) {
		std::cout << "ERROR: 读取路径错误" << std::endl;
		return;
	}

	// 清空所有数据
	std::vector<Eigen::Vector3f>().swap(*vertices); // 所有顶点
	std::vector<std::vector<int>>().swap(*faces); // 所有面
	std::vector<float>().swap(*vertex_data);
	std::vector<float>().swap(*color_data);
	std::vector<std::vector<std::vector<int>>>().swap(faces_on_grid);
	std::map<std::vector<int>, int>().swap(grid_to_vertex);

	/************ 读取全局信息 ************/
	// N_bins, patch总数
	infile >> N_bins >> patch_num;
	int feature_len = N_bins * N_bins;
	infile.get();
	// patch特征
	std::vector<Eigen::MatrixXf> patch_dictionaries;
	std::vector<Eigen::MatrixXf> patch_codes;
	int total_features;
	infile >> total_features;
	for (int i = 0; i < total_features; ++i) {
		// 算子数
		int atoms;
		infile >> atoms;
		// 字典
		Eigen::MatrixXf dictionary(feature_len, atoms);
		for (int i = 0; i < dictionary.rows(); ++i) {
			for (int j = 0; j < dictionary.cols(); ++j) {
				infile >> dictionary(i, j);
			}
		}
		patch_dictionaries.push_back(std::move(dictionary));
		// 编码
		Eigen::MatrixXf code(atoms, patch_num);
		for (int i = 0; i < code.rows(); ++i) {
			for (int j = 0; j < code.cols(); ++j) {
				infile >> code(i, j);
			}
		}
		patch_codes.push_back(std::move(code));
	}
	infile.get();
	// patch间连接性
	int crackface_num;
	infile >> crackface_num;
	infile.get();
	for (int i = 0; i < crackface_num; ++i) {
		int patch0, grid0, patch1, grid1, patch2, grid2;
		char c;
		infile >> patch0 >> c >> grid0
			>> patch1 >> c >> grid1
			>> patch2 >> c >> grid2;
		faces_on_grid.push_back({ {patch0, grid0}, {patch1, grid1}, {patch2, grid2} });
	}
	infile.get();

	/************ 读取patch信息 ************/
	std::vector<Eigen::Vector3f> patch_cord; // 种子点坐标
	std::vector<Eigen::Vector3f> patch_norm; // 种子点法线
	std::vector<float> patch_grid_span; // patch网格的尺寸
	std::vector<Eigen::Vector2f> patch_seed_bias; // 采样网格的位移
	std::vector<std::vector<int>> patch_mask; // 掩码
	for (int patch_index = 0; patch_index < patch_num; ++patch_index) {
		// 坐标
		Eigen::Vector3f seed_cord;
		infile >> seed_cord[0] >> seed_cord[1] >> seed_cord[2];
		patch_cord.push_back(seed_cord);

		// 法线
		Eigen::Vector3f seed_norm;
		infile >> seed_norm[0] >> seed_norm[1] >> seed_norm[2];
		patch_norm.push_back(seed_norm);

		// 网格尺寸和原点偏移
		float grid_span, x_bias, y_bias;
		infile >> grid_span >> x_bias >> y_bias;
		patch_grid_span.push_back(grid_span);
		patch_seed_bias.emplace_back(x_bias, y_bias);

		// 掩码
		int size;
		infile >> size;
		std::vector<int> mask(size);
		for (int i = 0; i < size; ++i) {
			infile >> mask[i];
		}
		patch_mask.push_back(std::move(mask));

		// patch内连接性
		int face_num;
		infile >> face_num;
		for (int i = 0; i < face_num; ++i) {
			int grid0, grid1, grid2;
			infile >> grid0 >> grid1 >> grid2;
			faces_on_grid.push_back({ {patch_index, grid0}, {patch_index, grid1}, {patch_index, grid2} });
		}

		// patch间连接性，但有两个顶点属于同一patch
		int record_num;
		infile >> record_num;
		for (int i = 0; i < record_num; ++i) {
			int grid0, grid1, patch2, grid2;
			char c;
			infile >> grid0 >> grid1 >> patch2 >> c >> grid2;
			faces_on_grid.push_back({ {patch_index, grid0}, {patch_index, grid1}, {patch2, grid2} });
		}

		infile.get();
	}
	infile.close();

	/************ 处理patch信息 ************/
	std::vector<int>(patch_num, 1).swap(patch_size); // 记录patch所包含的顶点数，主要用于调试
	std::vector<int>().swap(vertex_to_patch); // 记录顶点号到patch号的映射，主要用于调试
	std::vector<std::vector<int>>(patch_num).swap(patch_faces); // patch所包含的面号，主要用于调试

	// 还原顶点
	atoms = patch_dictionaries[0].cols(); // 因为现在只用到了一个特征，所以直接用下标0来获取特征数据了。但是可以看到显然本方法支持读取多个特征，使用不同下标即可
	Eigen::MatrixXf patch_grid_height = patch_dictionaries[0] * patch_codes[0];
	assert(patch_grid_height.rows() == feature_len);
	assert(patch_grid_height.cols() == patch_num);
	

	for (int patch_index = 0; patch_index < patch_num; ++patch_index) {
		Eigen::Vector3f seed_cord = patch_cord[patch_index];
		Eigen::Vector3f seed_norm = patch_norm[patch_index];
		Eigen::Matrix4f transform = Compressor::generate_transform(seed_cord, seed_norm);
		map_grid_to_vertex(patch_index, -1, seed_cord); // 先记录种子点，种子点不包含在grid里

		float grid_span = patch_grid_span[patch_index];
		float base_x = -grid_span * N_bins / 2.0f, base_y = base_x;

		for (int grid : patch_mask[patch_index]) {
			patch_size[patch_index] += 1;
			int grid_y = grid / N_bins;
			int grid_x = grid % N_bins;
			float x = base_x + (grid_x + 0.5f) * grid_span + patch_seed_bias[patch_index][0];
			float y = base_y + (grid_y + 0.5f) * grid_span + patch_seed_bias[patch_index][1];
			float height = patch_grid_height(grid, patch_index);
			Eigen::Vector4f point_cord = transform.inverse() * Eigen::Vector4f(x, y, height, 1.0f);
			if (point_cord[3] != 0) {
				point_cord /= point_cord[3];
			}
			map_grid_to_vertex(patch_index, grid, Eigen::Vector3f(point_cord[0], point_cord[1], point_cord[2]));
		}
	}
	// 还原面
	for (const auto& face : faces_on_grid) {
		int patch0 = face[0][0], grid0 = face[0][1];
		int patch1 = face[1][0], grid1 = face[1][1];
		int patch2 = face[2][0], grid2 = face[2][1];
		assert(grid_to_vertex.find({ patch0, grid0 }) != grid_to_vertex.end());
		int v0 = grid_to_vertex[{ patch0, grid0 }];
		assert(grid_to_vertex.find({ patch1, grid1 }) != grid_to_vertex.end());
		int v1 = grid_to_vertex[{ patch1, grid1 }];
		assert(grid_to_vertex.find({ patch2, grid2 }) != grid_to_vertex.end());
		int v2 = grid_to_vertex[{ patch2, grid2 }];
		faces->push_back({ v0, v1, v2 });
		if (patch0 == patch1 && patch0 == patch2) { // patch内的三角形
			patch_faces[patch0].push_back(faces->size() - 1);
		}

		// 顺便写坐标数组和颜色数组
		auto get_color = [](int i, int total) -> Eigen::Vector3f {
			if (total == 1) return Eigen::Vector3f(1.0f, 0.0f, 0.0f);
			float value = (2.0f / (total - 1)) * i;
			value = std::min(value, 2.0f);
			float r = 0.0f, g = 0.0f, b = 0.0f;
			if (value <= 1.0f) {
				r = 1.0 - value;
				g = value;
			}
			else {
				g = 2.0f - value;
				b = value - 1.0f;
			}
			return Eigen::Vector3f(r, g, b);
		};
		std::vector<int> triangle = { v0, v1, v2 };
		std::vector<int> patch = { patch0, patch1, patch2 };
		for (int i = 0; i < 3; ++i) {
			int point_index = triangle[i];
			vertex_data->push_back(vertices->at(point_index)[0]);
			vertex_data->push_back(vertices->at(point_index)[1]);
			vertex_data->push_back(vertices->at(point_index)[2]);

			Eigen::Vector3f color = get_color(patch[i], patch_num);
			color_data->push_back(color[0]);
			color_data->push_back(color[1]);
			color_data->push_back(color[2]);
		}
	}
}

void Parser::init(std::vector<Eigen::Vector3f>* _vertices, std::vector<std::vector<int>>* _faces, std::vector<float>* _vertex_data, std::vector<float>* _color_data) {
	vertices = _vertices;
	faces = _faces;
	vertex_data = _vertex_data;
	color_data = _color_data;
}

void Parser::write_patch_info(const std::vector<std::vector<int>>*& _patch_faces, const std::vector<int>*& _vertex_to_patch, 
	const std::vector<int>*& _patch_size, int& _feature_len, int& _atoms) {
	_patch_faces = &patch_faces;
	_vertex_to_patch = &vertex_to_patch;
	_patch_size = &patch_size;
	_feature_len = N_bins * N_bins;
	_atoms = atoms;
}