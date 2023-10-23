#pragma once

#include <core/core.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <learnopengl/shader.h>
#include <learnopengl/camera.h>

class PolygonPicker {
public:
	PolygonPicker();
	~PolygonPicker();

	void init(const std::vector<Eigen::Vector3f>* _vertices, const std::vector<std::vector<int>>* _faces, const std::vector<float>* _color_data, 
		const unsigned* _vbo_color, const Camera* _camera);
	// 射线检测选择三角形
	int select_triangle(float xpos, float ypos, unsigned window_width, unsigned window_height, const glm::mat4& projection, const glm::mat4& view, 
		const glm::vec3& camera_pos); 
	// 绘制射线检测的射线
	void draw_ray(const glm::mat4& model, const glm::mat4& view, const glm::mat4& projection);
	// 更新颜色缓存
	void update_color(int picked_face, int picked_patch, const vector<int>* patch_faces); 
	// patch相关
	void read_patch_info(const std::vector<std::vector<int>>* _patch_faces, const std::vector<int>* _vertex_to_patch, const std::vector<int>* _patch_size, int _feature_len, int _atoms); 
	int get_vertices_num() { return vertices->size(); }
	int get_faces_num() { return faces->size(); }
	int get_patch_num() { return patch_faces->size(); }
	int get_feature_len() { return feature_len; }
	int get_atoms() { return atoms; }
	// 根据面号查找其所在patch的所有其他面
	std::vector<std::vector<int>> get_patch_info(int query_face_id, std::vector<int>* faces_in_same_patch); 

private:
	unsigned vao_ray = 0;
	unsigned vbo_ray = 0;
	const unsigned* vbo_color = 0;

	Shader* ray_shader;
	const Camera* camera;
	const std::vector<Eigen::Vector3f>* vertices;
	const std::vector<std::vector<int>>* faces;
	const std::vector<float>* color_data;
	
	int selected_face = -1;
	std::vector<float> face_highlight_color;
	std::vector<float> face_backup_color;

	int selected_patch = -1;
	std::vector<int> selected_patch_faces;
	std::vector<float> patch_highlight_color;
	std::vector<std::vector<float>> patch_backup_color;

	const std::vector<std::vector<int>>* patch_faces; // 记录patch所包含的面号，主要用于调试
	const std::vector<int>* vertex_to_patch; // 记录顶点号到patch号的映射，主要用于调试
	const std::vector<int>* patch_size; // 记录patch所包含的顶点数，主要用于调试
	int feature_len; // 记录特征长度，主要用于调试
	int atoms; // 记录算子数量，主要用于调试

	std::vector<float> ray_data;
	
	// 根据鼠标点击位置获取射线方向
	glm::vec3 get_ray_orient(float xpos, float ypos, unsigned window_width, unsigned window_height,
		const glm::mat4& projection, const glm::mat4& view, const glm::vec3& camera_pos); 
	// 计算射线与三角形的交点
	bool intersect_triangle(const glm::vec3& orig, const glm::vec3& dir, 
		const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, float* t, float* u, float* v);

};