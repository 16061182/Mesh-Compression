#include "compressor.h"

#include <cmath>
#include <numeric>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>

Compressor::Compressor() {
}

Compressor::~Compressor() {
}

void Compressor::init(const std::vector<Eigen::Vector3f>* _vertices, const std::vector<std::vector<int>>* _faces, 
	const std::vector<Eigen::Vector3f>* _normals, int _N_bins, int _patch_size_limit, float _patch_normal_tolerance, int _float_precision) {
	origin_vertices = _vertices;
	origin_faces = _faces;
	origin_normals = _normals;

	std::vector<int>(_vertices->size(), -1).swap(vertex_to_patch);
	std::vector<int>(_vertices->size(), -1).swap(vertex_to_grid);
	std::vector<float>(_vertices->size()).swap(vertex_curvature);

	N_bins = _N_bins;
	patch_size_limit = _patch_size_limit;
	patch_normal_tolerance = _patch_normal_tolerance;
	float_precision = _float_precision;
}

void Compressor::generate_edge_parameter() {
	for (const auto& face : *origin_faces) {
		Eigen::Vector3f v0 = origin_vertices->at(face[0]);
		Eigen::Vector3f v1 = origin_vertices->at(face[1]);
		Eigen::Vector3f v2 = origin_vertices->at(face[2]);

		Eigen::Vector3f n0 = origin_normals->at(face[0]);
		Eigen::Vector3f n1 = origin_normals->at(face[1]);
		Eigen::Vector3f n2 = origin_normals->at(face[2]);

		float dis01 = (v0 - v1).norm();
		float dis02 = (v0 - v2).norm();
		float dis12 = (v1 - v2).norm();

		float curvature01 = (n0 - n1).dot(v0 - v1) / (v0 - v1).squaredNorm();
		float curvature02 = (n0 - n2).dot(v0 - v2) / (v0 - v2).squaredNorm();
		float curvature12 = (n1 - n2).dot(v1 - v2) / (v1 - v2).squaredNorm();

		// 距离
		edge_parameter[face[0]][face[1]].push_back(dis01);
		edge_parameter[face[0]][face[2]].push_back(dis02);
		edge_parameter[face[1]][face[0]].push_back(dis01);
		edge_parameter[face[1]][face[2]].push_back(dis12);
		edge_parameter[face[2]][face[0]].push_back(dis02);
		edge_parameter[face[2]][face[1]].push_back(dis12);

		// 曲率
		edge_parameter[face[0]][face[1]].push_back(curvature01);
		edge_parameter[face[0]][face[2]].push_back(curvature02);
		edge_parameter[face[1]][face[0]].push_back(curvature01);
		edge_parameter[face[1]][face[2]].push_back(curvature12);
		edge_parameter[face[2]][face[0]].push_back(curvature02);
		edge_parameter[face[2]][face[1]].push_back(curvature12);
	}
}

void Compressor::generate_patches() {
	std::vector<std::vector<int>>().swap(patch_vertices);
	std::vector<int>().swap(patch_size);
	int vertices_num = origin_vertices->size();
	
	// 计算边的长度和曲率
	generate_edge_parameter();

	// 首先对顶点按照(高斯)曲率的绝对值进行降序排序，这里是估算值，因为理论上最大曲率切面和最小曲率切面应该垂直
	std::vector<int> vertex_rank(vertices_num); // 按照曲率从大到小进行排序
	for (int i = 0; i < vertices_num; ++i) {
		float max_curvature = float(-2e9), min_curvature = float(2e9); // 相邻边的曲率中的最大者和最小者
		for (const auto& edge : edge_parameter[i]) {
			max_curvature = std::max(max_curvature, edge.second[1]);
			min_curvature = std::min(min_curvature, edge.second[1]);
		}
		vertex_curvature[i] = max_curvature * min_curvature;
		vertex_rank[i] = i;
	}
	auto comp = [&](int a, int b) -> bool {
		return abs(vertex_curvature[a]) > abs(vertex_curvature[b]);
	};
	sort(vertex_rank.begin(), vertex_rank.end(), comp);

	// 生成patch
	std::vector<bool> covered(vertices_num, false); // 是否已被seed覆盖，已被覆盖的点可以再次被覆盖，但不能作为新的seed
	while (true) {
		// 寻找新seed
		int next_seed = 0;
		while (next_seed < vertices_num && covered[vertex_rank[next_seed]]) ++next_seed;
		if (next_seed == vertices_num) break;
		int seed_id = vertex_rank[next_seed];

		// 记录新seed
		int patch_id = patch_vertices.size();
		patch_vertices.push_back({seed_id}); // 种子点(seed)是patch的第一个顶点
		vertex_to_patch[seed_id] = patch_id;
		covered[seed_id] = true;

		// 使用bfs泛洪法生成patch
		std::list<int> border; // 扩展边界，即bfs队列
		std::set<int> patch;
		border.push_back(seed_id);
		patch.insert(seed_id);
		bool full = false; // patch规模不超过N_bins * N_bins
		float pi = atan2(0.0f, -1.0f);
		float cos_tolerance = cos(patch_normal_tolerance * pi / 180);

		while (border.size() != 0 && !full) {
			int vertex = border.front();
			border.pop_front();
			for (const auto& connection : edge_parameter[vertex]) {
				int to_vertex = connection.first;
				if (covered[to_vertex]) continue;
				if (origin_normals->at(seed_id).dot(origin_normals->at(to_vertex)) <= cos_tolerance) continue; // 对法线方向做约束，与种子点法线的夹角不超过90度
				border.push_back(to_vertex);
				patch.insert(to_vertex);

				patch_vertices[patch_id].push_back(to_vertex);
				vertex_to_patch[to_vertex] = patch_id;
				covered[to_vertex] = true;

				if (patch.size() >= patch_size_limit) {
					full = true;
					break;
				}
			}
		}
		// TODO:邻居很少的点不能作为种子
		// 因为输入不是扫描得到的点云，本身就比较规整，所以暂且忽略这一步
	}

	// 记录patch数量
	patch_num = patch_vertices.size();
	for (const auto& it : patch_vertices) {
		patch_size.push_back(it.size());
	}
}

void Compressor::generate_patch_color(std::vector<float>* color_data) {
	// 先划分patch
	if (patch_vertices.size() == 0) generate_patches();

	// 根据种子点曲率的大小确定颜色
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

	std::vector<float>().swap(*color_data);

	for (const auto& face : *origin_faces) {
		for (int i = 0; i < 3; ++i) {
			int patch_index = vertex_to_patch[face[i]];
			Eigen::Vector3f patch_color = get_color(patch_index, patch_num);
			color_data->push_back(patch_color[0]);
			color_data->push_back(patch_color[1]);
			color_data->push_back(patch_color[2]);
		}
	}
	//check(0);
}

Eigen::Matrix4f Compressor::generate_transform(const Eigen::Vector3f& cord, const Eigen::Vector3f& in_normal) {
	Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();
	translate(0, 3) = -cord[0];
	translate(1, 3) = -cord[1];
	translate(2, 3) = -cord[2];

	Eigen::Vector3f normal = in_normal.normalized(); // 原始法线可能未单位化
	Eigen::Vector3f cand_tangent = { 1.0f, 0.0f, 0.0f }; // 取世界坐标x轴作为候选tangent
	if (normal.cross(cand_tangent).norm() < 1e-5) {
		cand_tangent = { 0.0f, 1.0f, 0.0f }; // 若恰好平行，改为世界y轴
	}
	// 格拉姆-施密特正交化
	Eigen::Vector3f tangent = (cand_tangent - (cand_tangent.dot(normal) / normal.squaredNorm()) * normal).normalized();
	Eigen::Vector3f bitangent = normal.cross(tangent).normalized();
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
	for (int i = 0; i < 3; ++i) {
		view(i, 0) = tangent[i];
		view(i, 1) = bitangent[i];
		view(i, 2) = normal[i];
	}
	view.transposeInPlace();

	return view * translate;
}

void Compressor::resample() {
	Eigen::MatrixXf patch_resample_height = Eigen::MatrixXf::Constant(N_bins * N_bins, patch_num, 0.0f);
	std::vector<std::vector<int>>(patch_num).swap(patch_masks);
	std::vector<float>(patch_num).swap(patch_grid_span);
	std::vector<Eigen::Vector2f>(patch_num).swap(patch_seed_bias);

	for (int patch_id = 0; patch_id < patch_num; ++patch_id) {
		int seed_id = patch_vertices[patch_id][0];
		std::vector<std::vector<float>> grid_vertex_height(N_bins * N_bins); // 多个顶点可能被采样到同一个网格，用一个映射记录每个网格包含的顶点
		Eigen::Matrix4f transform = generate_transform(origin_vertices->at(seed_id), origin_normals->at(seed_id));

		// 计算顶点的局部坐标
		std::vector<Eigen::Vector3f> local_cord_record;
		float min_x = 0.0f, max_x = 0.0f, min_y = 0.0f, max_y = 0.0f;
		for (int i = 1; i < patch_vertices[patch_id].size(); ++i) {
			int point = patch_vertices[patch_id][i];
			Eigen::Vector4f point_cord = { origin_vertices->at(point)[0], origin_vertices->at(point)[1], origin_vertices->at(point)[2], 1.0f };
			Eigen::Vector4f local_cord = transform * point_cord;
			if (local_cord[3] != 0) {
				local_cord /= local_cord[3]; // 齐次化
			}
			local_cord_record.emplace_back(local_cord[0], local_cord[1], local_cord[2]);
			min_x = std::min(min_x, local_cord[0]);
			max_x = std::max(max_x, local_cord[0]);
			min_y = std::min(min_y, local_cord[1]);
			max_y = std::max(max_y, local_cord[1]);
		}
		assert(local_cord_record.size() == patch_vertices[patch_id].size() - 1);
		
		// 记录网格重采样的高度
		Eigen::Vector2f new_grid_origin((min_x + max_x) / 2, (min_y + max_y) / 2);
		float reach = (max_x - min_x > max_y - min_y) ? (max_x - min_x) : (max_y - min_y);
		reach = reach * N_bins / (N_bins - 1); // 放缩
		float span = reach / N_bins;
		patch_grid_span[patch_id] = span; // 每个patch分别记录网格大小
		patch_seed_bias[patch_id] = new_grid_origin;
		float base_x = -reach / 2, base_y = base_x;
		for (int i = 0; i < local_cord_record.size(); ++i) {
			float x = local_cord_record[i][0] - new_grid_origin[0];
			float y = local_cord_record[i][1] - new_grid_origin[1];
			assert(x > -reach / 2 && x < reach / 2);
			assert(y > -reach / 2 && y < reach / 2);
			int x_grid = (x - base_x) / span;
			int y_grid = (y - base_y) / span;
			assert(x_grid >= 0 && x_grid < N_bins);
			assert(y_grid >= 0 && y_grid < N_bins);
			grid_vertex_height[N_bins * y_grid + x_grid].push_back(local_cord_record[i][2]);
			// 记录顶点所属的grid
			int point = patch_vertices[patch_id][i + 1]; // 每个patch的第一个顶点是seed，seed已经记过了，从第二个顶点开始记
			vertex_to_grid[point] = N_bins * y_grid + x_grid;
		}

		// 记录grid里所有顶点的平均local高度作为该grid的采样高度
		for (int grid = 0; grid < N_bins * N_bins; ++grid) {
			if (!grid_vertex_height[grid].empty()) {
				float total_height = std::accumulate(grid_vertex_height[grid].begin(), grid_vertex_height[grid].end(), 0.0f);
				patch_resample_height(grid, patch_id) = total_height / grid_vertex_height[grid].size();
				patch_masks[patch_id].push_back(grid); // 记录采样到了顶点的网格(这些网格在解压缩时需要还原成对应的顶点)
			}
		}
	}
	patch_featuress.push_back(std::move(patch_resample_height));
}

void Compressor::coding(const Eigen::MatrixXf& feature, Eigen::MatrixXf& dictionary, Eigen::MatrixXf& code, int _atoms) {
	// 默认值
	int new_atoms = _atoms;
	if (new_atoms <= 0) {
		new_atoms = feature.cols();
	}
	// 使用SVD分解，得到字典和编码
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(feature, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::VectorXf S = svd.singularValues();
	if (new_atoms > svd.matrixU().cols()) {
		new_atoms = svd.matrixU().cols();
	}
	if (new_atoms != _atoms) {
		std::cout << "LOG: 算子数量重新调整为" << new_atoms << std::endl;
		_atoms = new_atoms;
	}
	Eigen::MatrixXf clipU = svd.matrixU()(Eigen::all, Eigen::seqN(0, _atoms)); // 前atoms列
	Eigen::MatrixXf clipV = svd.matrixV()(Eigen::all, Eigen::seqN(0, _atoms)); // 前atoms行(注意后面有转置操作，因此这里取前atoms列)
	Eigen::MatrixXf diagS = Eigen::MatrixXf::Identity(_atoms, _atoms);
	diagS.diagonal(0) = S(Eigen::seqN(0, _atoms)); // diagS的主对角线是前atoms个奇异值，已经按照从大到小排列
	dictionary = clipU;

	code = diagS * clipV.transpose(); // 与np.linalg.svd不同，Eigen::JacobiSVD的V是转置之前的，使用时需转置
}

void Compressor::record_connection() {
	auto comp = [](const std::vector<int>& a, const std::vector<int>& b) -> bool {
		return a[0] == b[0] ? a[1] < b[1] : a[0] < b[0];
	};

	std::vector<std::set<std::vector<int>>>(patch_num).swap(patch_faces);
	std::vector<std::vector<int>>(patch_num).swap(patch_origin_faces);
	std::vector<std::set<std::vector<int>>>(patch_num).swap(bi_crackfaces);
	for (int i = 0; i < origin_faces->size(); ++i) {
		int v0 = origin_faces->at(i)[0];
		int v1 = origin_faces->at(i)[1];
		int v2 = origin_faces->at(i)[2];

		std::vector<std::vector<int>> patch_grid = {
			{ vertex_to_patch[v0], vertex_to_grid[v0] },
			{ vertex_to_patch[v1], vertex_to_grid[v1] },
			{ vertex_to_patch[v2], vertex_to_grid[v2] }
		};

		sort(patch_grid.begin(), patch_grid.end(), comp);

		int patch0 = patch_grid[0][0], grid0 = patch_grid[0][1]; // 已经排好序
		int patch1 = patch_grid[1][0], grid1 = patch_grid[1][1];
		int patch2 = patch_grid[2][0], grid2 = patch_grid[2][1];

		if (patch0 == patch1 && patch0 == patch2) { // 三个顶点属于同一个patch
			patch_origin_faces[patch0].push_back(i);
			if (grid0 != grid1 && grid0 != grid2 && grid1 != grid2) { // 三个顶点分属不同网格
				patch_faces[patch0].insert({ grid0, grid1, grid2 });
			}
		}
		else if (patch0 != patch1 && patch0 != patch2 && patch1 != patch2) {
			tri_crackfaces.insert(patch_grid);
		}
		else {
			if (patch0 == patch1) {
				bi_crackfaces[patch0].insert({ grid0, grid1, patch2, grid2 }); // 由于前面已经排好序，因此grid0一定小于grid1，下同
			}
			else if (patch1 == patch2) {
				bi_crackfaces[patch1].insert({ grid1, grid2, patch0, grid0 });
			}
			else {
				bi_crackfaces[patch0].insert({ grid0, grid2, patch1, grid1 });
			}
		}
	}
}

void Compressor::serialize(std::string save_path) {
	std::ofstream outfile(save_path, std::ios::ate);
	if (!outfile.is_open()) {
		std::cout << "ERROR: 保存路径错误" << std::endl;
		return ;
	}
	outfile << std::dec << std::fixed << std::setprecision(float_precision);

	/************ 全局信息 ************/
	// N_bins, patch总数
	outfile << N_bins << ' ' << patch_num << std::endl;
	outfile << std::endl;
	// patch特征
	outfile << patch_featuress.size() << std::endl;
	for (int i = 0; i < patch_featuress.size(); ++i) {
		// 算子数
		outfile << patch_atoms[i] << std::endl;
		// 字典
		const auto& dictionary = patch_dictionaries[i];
		for (int row = 0; row < dictionary.rows(); ++row) {
			for (int col = 0; col < dictionary.cols() - 1; ++col) {
				outfile << dictionary(row, col) << ' ';
			}
			outfile << dictionary(row, dictionary.cols() - 1) << std::endl;
		}
		// 编码
		const auto& code = patch_codes[i];
		for (int row = 0; row < code.rows(); ++row) {
			for (int col = 0; col < code.cols() - 1; ++col) {
				outfile << code(row, col) << ' ';
			}
			outfile << code(row, code.cols() - 1) << std::endl;
		}
	}
	outfile << std::endl;
	// patch间连接性
	outfile << tri_crackfaces.size() << std::endl;
	for (const auto& face : tri_crackfaces) {
		outfile << face[0][0] << '/' << face[0][1] << ' ' << face[1][0] << '/' << face[1][1] << ' ' << face[2][0] << '/' << face[2][1] << std::endl;
	}
	outfile << std::endl;

	/************ patch信息 ************/
	for (int patch_id = 0; patch_id < patch_num; ++patch_id) {
		//int patch_id = patch_index_map_reverse[i];
		int seed_id = patch_vertices[patch_id][0];
		// 坐标
		outfile << origin_vertices->at(seed_id)[0] << ' ' << origin_vertices->at(seed_id)[1] << ' ' << origin_vertices->at(seed_id)[2] << std::endl;
		// 法线
		outfile << origin_normals->at(seed_id)[0] << ' ' << origin_normals->at(seed_id)[1] << ' ' << origin_normals->at(seed_id)[2] << std::endl;
		// 网格尺寸和原点偏移
		outfile << patch_grid_span[patch_id] << ' ' << patch_seed_bias[patch_id][0] << ' ' << patch_seed_bias[patch_id][1] << std::endl;
		// 掩码
		int size = patch_masks[patch_id].size();
		outfile << size << std::endl;
		if (size > 0) {
			for (int i = 0; i < size - 1; ++i) {
				outfile << patch_masks[patch_id][i] << ' ';
			}
			outfile << patch_masks[patch_id][size - 1] << std::endl;
		}
		// patch内连接性
		outfile << patch_faces[patch_id].size() << std::endl;
		for (const auto& face : patch_faces[patch_id]) {
			outfile << face[0] << ' ' << face[1] << ' ' << face[2] << std::endl;
		}
		// patch间连接性，但有两个顶点属于同一patch
		outfile << bi_crackfaces[patch_id].size() << std::endl;
		for (const auto& record : bi_crackfaces[patch_id]) {
			outfile << record[0] << ' ' << record[1] << ' ' << record[2] << '/' << record[3] << std::endl;
		}
		outfile << std::endl;
	}

	outfile.close();
}

void Compressor::compress_and_save(int _atoms, const std::string& save_path) {
	if (patch_vertices.size() == 0) generate_patches();
	resample();
	for (int i = 0; i < patch_featuress.size(); ++i) {
		Eigen::MatrixXf dictionary, code;
		coding(patch_featuress[i], dictionary, code, _atoms);
		patch_atoms.push_back(dictionary.cols());
		patch_dictionaries.push_back(std::move(dictionary));
		patch_codes.push_back(std::move(code));
	}
	record_connection();
	serialize(save_path);
	//check(1);
	//check(2);
	//check(3);

}

void Compressor::check(int part) {
	std::cout << std::endl;
	std::cout << "----------check " << part << "----------" << std::endl;
	if (part == 0) {
		std::vector<int> patch_size;
		for (const auto& it : patch_vertices) {
			patch_size.push_back(it.size());
		}
		sort(patch_size.begin(), patch_size.end());
		for (int i : patch_size) {
			std::cout << i << " ";
		}
		std::cout << std::endl;

		float average = accumulate(patch_size.begin(), patch_size.end(), 0.0f) * 1.0f / patch_num;
		float medium;
		if (patch_num % 2) {
			medium = patch_size[patch_num / 2];
		}
		else {
			medium = (patch_size[patch_num / 2] + patch_size[patch_num / 2 - 1]) / 2.0f;
		}
		std::cout << "patch数量: " << patch_num << std::endl;
		std::cout << "patch规模平均数: " << average << std::endl;
		std::cout << "patch规模中位数: " << medium << std::endl;
	}
	else if (part == 1) {
		int count = 0;
		for (int i = 0; i < origin_vertices->size(); ++i) {
			if (vertex_to_patch[i] == -1) {
				std::cout << "点" << i << "没有patch" << std::endl;
			}
			if (vertex_to_grid[i] == -1) {
				count += 1;
			}
		}
		std::cout << "patch数量: " << patch_num << std::endl;
		std::cout << "seed数量" << count << std::endl;
		int cnt = 0;
		for (int i = 0; i < patch_num; ++i) {
			int seed_id = patch_vertices[i][0];
			if (vertex_to_grid[seed_id] != -1) {
				std::cout << "patch" << i << "的种子" << seed_id << "属于网格" << vertex_to_grid[seed_id] << std::endl;
				cnt += 1;
			}
		}
		std::cout << "cnt: " << cnt << std::endl;
	}
	else if (part == 2) {
		int count = 0;
		int sum = 0;
		for (int i = 0; i < patch_num; ++i) {
			if (patch_vertices[i].size() == patch_size_limit) {
				sum += 1;
				count += patch_masks[i].size();
			}

		}
		float average = count * 1.0f / sum;
		std::cout << "满patch的平均有值网格数: " << average << std::endl;
		std::cout << "平均有值网格占比: " << average / patch_size_limit * 100 << "%" << std::endl; // 除32而不是64
	}
	else if (part == 3) {
		std::vector<bool> check(origin_vertices->size(), false);
		std::set<int> rec;
		for (const auto& it : patch_vertices) {
			for (int i : it) {
				check[i] = true;
				if (rec.find(i) != rec.end()) {
					std::cout << "顶点" << i << "重复了！" << std::endl;
				}
				rec.insert(i);
			}
		}
	}
	std::cout << std::endl;
}

void Compressor::write_patch_info(const std::vector<std::vector<int>>*& _patch_faces, const std::vector<int>*& _vertex_to_patch, 
	const std::vector<int>*& _patch_size, int& _feature_len, int& _atoms) {
	_patch_faces = &patch_origin_faces;
	_vertex_to_patch = &vertex_to_patch;
	_patch_size = &patch_size;
	_feature_len = N_bins * N_bins;
	_atoms = patch_atoms[0];
}
