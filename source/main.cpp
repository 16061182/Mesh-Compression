#define STB_IMAGE_IMPLEMENTATION // 通过定义STB_IMAGE_IMPLEMENTATION，预处理器会修改头文件，让其只包含相关的函数定义源码，等于是将这个头文件变为一个 .cpp 文件了。
#define TINYOBJLOADER_IMPLEMENTATION
//#define TINYOBJLOADER_USE_MAPBOX_EARCUT

#include <iostream>
#include <thread>
#include <memory>
#include <chrono>

#include <core/data.h>
#include <tools/load_obj_mesh.h>
#include <display/polygon_picker.h>
#include <algorithm/compressor.h>
#include <algorithm/parser.h>
#include <display/opengl_window.h>

using namespace std;

void thread_func(bool recovered, shared_ptr<Data> data) {
	string title = recovered ? "recovered" : "original";
	string vertex_shader = "resource/shader/model_loading_notex.vs";
	string fragment_shader = "resource/shader/model_loading_notex.fs";
	OpenGLWindow window(recovered, title, vertex_shader, fragment_shader, &data->vertex_data, &data->color_data);
	
	// 点选
	PolygonPicker polygon_picker;
	OpenGLWindow::polygon_picker = &polygon_picker;
	// 显示网格，点选功能
	window.bind_buffer();
	polygon_picker.init(&data->vertices, &data->faces, &data->color_data, &window.vbo_color, &OpenGLWindow::camera);
	polygon_picker.read_patch_info(data->patch_faces, data->vertex_to_patch, data->patch_size, data->feature_len, data->atoms);
	window.show();
}

int main() {
	shared_ptr<Data> original_data = make_shared<Data>();
	shared_ptr<Data> recovered_data = make_shared<Data>();
	Config config("config.json");

	// 加载
	ObjLoader obj_loader;
	std::string original_mesh_path = "resource/mesh/FinalBaseMesh.obj";
	obj_loader.init(&original_data->vertices, &original_data->faces, &original_data->normals, &original_data->vertex_data, &original_data->color_data);
	if (!obj_loader.load_obj_mesh(original_mesh_path)) {
		std::cout << "加载网格出错!" << endl;
		return -1;
	}
	std::string rewrite_mesh_path = "mesh_origin.data";
	obj_loader.rewrite_origin_mesh(rewrite_mesh_path);

	// 压缩
	Compressor compressor;
	std::string recovered_mesh_path = "mesh_compressed.data";
	compressor.init(&original_data->vertices, &original_data->faces, &original_data->normals, config.N_bins, config.patch_size_limit, config.patch_normal_tolerance, config.float_precision);
	compressor.generate_patch_color(&original_data->color_data);
	compressor.compress_and_save(config.atoms, recovered_mesh_path);
	compressor.write_patch_info(original_data->patch_faces, original_data->vertex_to_patch, original_data->patch_size, original_data->feature_len, original_data->atoms);
	//obj_loader.print_detail();

	// 解压缩
	Parser parser;
	parser.init(&recovered_data->vertices, &recovered_data->faces, &recovered_data->vertex_data, &recovered_data->color_data);
	parser.parse(recovered_mesh_path);
	parser.write_patch_info(recovered_data->patch_faces, recovered_data->vertex_to_patch, recovered_data->patch_size, recovered_data->feature_len, recovered_data->atoms);

	// 分别显示原始网格和压缩后还原的网格
	std::thread t1(thread_func, false, original_data);
	std::thread t2(thread_func, true, recovered_data);
	t1.join();
	t2.join();
	return 0;
}
