#include "load_obj_mesh.h"

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <fstream>
#include <iomanip>

ObjLoader::ObjLoader() {
}

ObjLoader::~ObjLoader() {
}

void ObjLoader::init(std::vector<Eigen::Vector3f>* _vertices, std::vector<std::vector<int>>* _faces, std::vector<Eigen::Vector3f>* _normals, 
	std::vector<float>* _vertex_data, std::vector<float>* _color_data) {
	vertices = _vertices;
	faces = _faces;
	normals = _normals;
	vertex_data = _vertex_data;
	color_data = _color_data;
}

bool ObjLoader::load_obj_mesh(const std::string& obj_file_path) {
	tinyobj::ObjReaderConfig reader_config;
	reader_config.triangulate = true; // 不进行三角剖分，保留多边形 // OpenGL不支持绘制多边形！必须进行三角剖分
	reader_config.vertex_color = false;
	reader_config.mtl_search_path = "./"; // Path to material files

	tinyobj::ObjReader reader;

	if (!reader.ParseFromFile(obj_file_path, reader_config)) {
		if (!reader.Error().empty()) {
			std::cerr << "TinyObjReader: " << reader.Error();
		}
		exit(1);
	}

	if (!reader.Warning().empty()) {
		std::cout << "TinyObjReader: " << reader.Warning();
	}

	auto& attrib = reader.GetAttrib();
	auto& shapes = reader.GetShapes();
	auto& materials = reader.GetMaterials();

	// 记录顶点全集
	size_t verts = attrib.vertices.size() / 3;
	for (size_t i = 0; i < verts; ++i) {
		vertices->push_back(Eigen::Vector3f(float(attrib.vertices[3 * i + 0]), float(attrib.vertices[3 * i + 1]), float(attrib.vertices[3 * i + 2])));
	}
	
	std::vector<Eigen::Vector3f>(verts, Eigen::Vector3f(0.0f, 0.0f, 0.0f)).swap(*normals);

	// Loop over shapes
	for (size_t s = 0; s < shapes.size(); s++) {
		// Loop over faces(polygon)
		size_t index_offset = 0;
		for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
			size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);

			// Loop over vertices in the face.
			std::vector<int> tmp;
			for (size_t v = 0; v < fv; v++) {
				// access to vertex
				tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
				tinyobj::real_t vx = attrib.vertices[3 * size_t(idx.vertex_index) + 0];
				tinyobj::real_t vy = attrib.vertices[3 * size_t(idx.vertex_index) + 1];
				tinyobj::real_t vz = attrib.vertices[3 * size_t(idx.vertex_index) + 2];

				vertex_data->push_back(float(vx));
				vertex_data->push_back(float(vy));
				vertex_data->push_back(float(vz));

				tmp.push_back(int(idx.vertex_index));

				// Check if `normal_index` is zero or positive. negative = no normal data
				if (idx.normal_index >= 0) {
					tinyobj::real_t nx = attrib.normals[3 * size_t(idx.normal_index) + 0];
					tinyobj::real_t ny = attrib.normals[3 * size_t(idx.normal_index) + 1];
					tinyobj::real_t nz = attrib.normals[3 * size_t(idx.normal_index) + 2];
					normals->at(idx.vertex_index) += Eigen::Vector3f(nx, ny, nz);
				}
			}
			faces->push_back(tmp);
			index_offset += fv;
		}
	}

	for (size_t i = 0; i < verts; ++i) {
		assert(normals->at(i).norm() > 0.0f);
		Eigen::Vector3f tmp = normals->at(i).normalized();
		normals->at(i) = tmp; // 不知道Eigen的原地normalized会不会像transpose一样出问题，所以这样写
	}

	std::cout << "LOG: 模型读取成功" << std::endl;
	calculate_scale();
	return true;
}

void ObjLoader::calculate_scale() {
	float total_len = 0.0f;
	for (const auto& face : *faces) {
		Eigen::Vector3f v0 = { vertices->at(face[0])[0], vertices->at(face[0])[1] , vertices->at(face[0])[2] };
		Eigen::Vector3f v1 = { vertices->at(face[1])[0], vertices->at(face[1])[1] , vertices->at(face[1])[2] };
		Eigen::Vector3f v2 = { vertices->at(face[2])[0], vertices->at(face[2])[1] , vertices->at(face[2])[2] };
		total_len += (v0 - v1).norm();
		total_len += (v0 - v2).norm();
		total_len += (v1 - v2).norm();
	}
	average_edge_len = total_len / faces->size() / 3;

	float min_x = float(2e9), max_x = float(-2e9), min_y = float(2e9), max_y = float(-2e9), min_z = float(2e9), max_z = float(-2e9);
	for (const auto& vertex : *vertices) {
		min_x = std::min(min_x, vertex[0]);
		max_x = std::max(max_x, vertex[0]);
		min_y = std::min(min_y, vertex[1]);
		max_y = std::max(max_y, vertex[1]);
		min_z = std::min(min_z, vertex[2]);
		max_z = std::max(max_z, vertex[2]);
	}
	scale_x = max_x - min_x;
	scale_y = max_y - min_y;
	scale_z = max_z - min_z;

}

void ObjLoader::print_detail() {
	std::cout << std::endl << "---------- 模型加载详情 ----------" << std::endl;
	std::cout << "顶点数: " << vertices->size() << std::endl;
	std::cout << "面数: " << faces->size() << std::endl;
	std::cout << "顶点数组长度: " << vertex_data->size() << std::endl;
	std::cout << "颜色数组长度: " << color_data->size() << std::endl;
	std::cout << "x轴坐标范围: " << scale_x << std::endl;
	std::cout << "y轴坐标范围: " << scale_y << std::endl;
	std::cout << "z轴坐标范围: " << scale_z << std::endl;
	std::cout << "边平均长度: " << average_edge_len << std::endl;
	std::cout << std::endl;
}

void ObjLoader::rewrite_origin_mesh(std::string save_path) {
	std::ofstream outfile(save_path, std::ios::ate);
	if (!outfile.is_open()) {
		std::cout << "ERROR: 保存路径错误" << std::endl;
		return;
	}

	outfile << std::dec << std::fixed << std::setprecision(float_precision);
	for (const auto& vertex : *vertices) {
		outfile << "v " << vertex[0] << " " << vertex[1] << " " << vertex[2] << std::endl;
	}
	outfile << std::endl;
	for (const auto& face : *faces) {
		outfile << "f " << face[0] << "// " << face[1] << "// " << face[2] << "//" << std::endl;
	}
	outfile.close();

}
