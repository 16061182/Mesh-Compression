#include "polygon_picker.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>

PolygonPicker::PolygonPicker() :
	face_backup_color(9, 0.0f),
	face_highlight_color(9, 0.0f), // 1.0f, 1.0f, 0.25f
	patch_highlight_color({
		1.0f, 1.0f, 0.25f,
		1.0f, 1.0f, 0.25f,
		1.0f, 1.0f, 0.25f
	}),
	ray_data({ 
		0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
	})
{
}

PolygonPicker::~PolygonPicker() {
	if (vao_ray != 0) {
		glDeleteVertexArrays(1, &vao_ray);
	}
	if (vbo_ray != 0) {
		glDeleteBuffers(1, &vao_ray);
	}
}

void PolygonPicker::init(const std::vector<Eigen::Vector3f>* _vertices, const std::vector<std::vector<int>>* _faces, const std::vector<float>* _color_data,
	const unsigned* _vbo_color, const Camera* _camera) {
	vertices = _vertices;
	faces = _faces;
	color_data = _color_data;
	vbo_color = _vbo_color;
	camera = _camera;

	ray_shader = new Shader("resource/shader/model_loading_notex.vs", "resource/shader/model_loading_notex.fs");

	glGenVertexArrays(1, &vao_ray);
	glGenBuffers(1, &vbo_ray);
}

void PolygonPicker::draw_ray(const glm::mat4& model, const glm::mat4& view, const glm::mat4& projection) {
	// 绘制射线
	glBindVertexArray(vao_ray);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_ray);
	glBufferData(GL_ARRAY_BUFFER, 12 * sizeof(float), &ray_data[0], GL_STATIC_DRAW);
	glVertexAttribPointer( // 位置
		0,        // attribute 0. No particular reason for 0, but must match the layout in the shader.
		3,        // size // 当前顶点属性由多少个float组成，如果是坐标的话就是3，如果是uv坐标的话就是2
		GL_FLOAT, // type
		GL_FALSE, // normalized?
		6 * sizeof(float),        // stride 
		(void*)0 // array buffer offset
	);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer( // 颜色
		1,        // attribute 0. No particular reason for 0, but must match the layout in the shader.
		3,        // size // 当前顶点属性由多少个float组成，如果是坐标的话就是3，如果是uv坐标的话就是2
		GL_FLOAT, // type
		GL_FALSE, // normalized?
		6 * sizeof(float),        // stride 
		(void*)(3 * sizeof(float)) // array buffer offset
	);
	glEnableVertexAttribArray(1);
	ray_shader->use();
	ray_shader->setMat4("projection", projection);
	ray_shader->setMat4("view", view);
	ray_shader->setMat4("model", model);

	glBindVertexArray(vao_ray);
	glLineWidth(2);
	glDrawArrays(GL_LINES, 0, 2);
}

glm::vec3 PolygonPicker::get_ray_orient(float xpos, float ypos, unsigned window_width, unsigned window_height,
	const glm::mat4& projection, const glm::mat4& view, const glm::vec3& camera_pos) {
	// 将屏幕坐标转换成NDC标准化坐标。注意屏幕坐标的原点在左上角
	float ndc_x = 2.0f * xpos / window_width - 1.0f;
	float ndc_y = 1.0f - (2.0f * ypos / window_width);

	// 远面上的鼠标点击位置
	glm::vec3 ray_end = glm::vec3(ndc_x, ndc_y, 1.0f); // z = 1.0f表示将鼠标的位置投影到远剪裁平面
	glm::vec4 ray_end_world = glm::inverse(view) * glm::inverse(projection) * glm::vec4(ray_end, 1.0f);
	// 手动齐次化
	if (ray_end_world.w != 0.0f) {
		ray_end_world.x = ray_end_world.x / ray_end_world.w;
		ray_end_world.y = ray_end_world.y / ray_end_world.w;
		ray_end_world.z = ray_end_world.z / ray_end_world.w;
	}

	// 近面上的鼠标点击位置 // 还有一种方法是用相机位置作为射线起点
	glm::vec3 ray_start = glm::vec3(ndc_x, ndc_y, -1.0f); // z = -1.0f是近平面吗？
	glm::vec4 ray_start_world = glm::inverse(view) * glm::inverse(projection) * glm::vec4(ray_start, 1.0f);
	// 手动齐次化
	if (ray_start_world.w != 0.0f) {
		ray_start_world.x = ray_start_world.x / ray_start_world.w;
		ray_start_world.y = ray_start_world.y / ray_start_world.w;
		ray_start_world.z = ray_start_world.z / ray_start_world.w;
	}

	// 记录射线的起点和终点
	for (int i = 0; i < 3; ++i) {
		ray_data[i] = ray_start_world[i];
		ray_data[6 + i] = ray_end_world[i];
	}


	glm::vec3 ray_orient = glm::normalize(glm::vec3(ray_end_world.x, ray_end_world.y, ray_end_world.z) - glm::vec3(ray_start_world.x, ray_start_world.y, ray_start_world.z));
	return ray_orient;
}

void PolygonPicker::update_color(int picked_face, int picked_patch, const vector<int>* patch_faces) {
	if (picked_patch != selected_patch) {
		// 还原颜色，只在上一次选中了patch的情况下进行
		if (selected_patch != -1) {
			glBindBuffer(GL_ARRAY_BUFFER, *vbo_color);
			for (int i = 0; i < selected_patch_faces.size(); ++i) {
				glBufferSubData(GL_ARRAY_BUFFER, selected_patch_faces[i] * 9 * sizeof(float), 9 * sizeof(float), &patch_backup_color[i][0]);
			}
		}
		// 保存和替换颜色，只在本次选中了patch的情况下进行
		if (picked_patch != -1) {
			// 保存颜色
			std::vector<std::vector<float>>(patch_faces->size(), std::vector<float>(9)).swap(patch_backup_color);
			for (int i = 0; i < patch_faces->size(); ++i) {
				for (int j = 0; j < 9; ++j) {
					patch_backup_color[i][j] = color_data->at(patch_faces->at(i) * 9 + j);
				}
			}
			// 替换颜色
			glBindBuffer(GL_ARRAY_BUFFER, *vbo_color);
			for (int face_id : *patch_faces) {
				glBufferSubData(GL_ARRAY_BUFFER, face_id * 9 * sizeof(float), 9 * sizeof(float), &patch_highlight_color[0]);
			}
		}
	}

	if (picked_face != selected_face) {
		// 还原颜色，只在上一次选中了三角形的情况下进行
		if (selected_face != -1) {
			glBindBuffer(GL_ARRAY_BUFFER, *vbo_color);
			if (picked_patch != -1 && picked_patch == selected_patch) { // 由于本patch仍在高亮中，所以不还原成原始颜色，而还原成patch高亮颜色
				glBufferSubData(GL_ARRAY_BUFFER, selected_face * 9 * sizeof(float), 9 * sizeof(float), &patch_highlight_color[0]);
			}
			else {
				glBufferSubData(GL_ARRAY_BUFFER, selected_face * 9 * sizeof(float), 9 * sizeof(float), &face_backup_color[0]);
			}
		}
		// 保存和替换颜色，只在本次选中了三角形的情况下进行
		if (picked_face != -1) {
			// 保存颜色
			for (int i = 0; i < 9; ++i) {
				face_backup_color[i] = color_data->at(picked_face * 9 + i);
			}
			// 替换颜色
			glBindBuffer(GL_ARRAY_BUFFER, *vbo_color);
			glBufferSubData(GL_ARRAY_BUFFER, picked_face * 9 * sizeof(float), 9 * sizeof(float), &face_highlight_color[0]);
		}
	}

	selected_face = picked_face;
	selected_patch = picked_patch;
	selected_patch_faces = *patch_faces;
}

void PolygonPicker::read_patch_info(const std::vector<std::vector<int>>* _patch_faces, const std::vector<int>* _vertex_to_patch, const std::vector<int>* _patch_size, int _feature_len, int _atoms) {
	patch_faces = _patch_faces;
	vertex_to_patch = _vertex_to_patch;
	patch_size = _patch_size;
	feature_len = _feature_len;
	atoms = _atoms;
}

std::vector<std::vector<int>> PolygonPicker::get_patch_info(int picked_face, std::vector<int>* faces_in_same_patch) {
	if (picked_face < 0 || picked_face >= faces->size()) return {};

	int v0 = faces->at(picked_face)[0];
	int v1 = faces->at(picked_face)[1];
	int v2 = faces->at(picked_face)[2];

	int patch0 = vertex_to_patch->at(v0);
	int patch1 = vertex_to_patch->at(v1);
	int patch2 = vertex_to_patch->at(v2);

	set<int> patch_set;
	patch_set.insert(patch0);
	patch_set.insert(patch1);
	patch_set.insert(patch2);
	vector<vector<int>> res;
	for (int patch_id : patch_set) {
		res.push_back({ patch_id, patch_size->at(patch_id) });
	}

	if (patch_set.size() == 1) {
		*faces_in_same_patch = patch_faces->at(patch0);
	}
	return res;
}

int PolygonPicker::select_triangle(float xpos, float ypos, unsigned window_width, unsigned window_height,
	const glm::mat4& projection, const glm::mat4& view, const glm::vec3& camera_pos) {
	glm::vec3 orient = get_ray_orient(xpos, ypos, window_width, window_height, projection, view, camera_pos);

	int res = -1;
	float lastt = INT_MAX;
	int faces_num = faces->size();
	for (int i = 0; i < faces_num; ++i) { // 注意因为使用的model矩阵是单位矩阵，所以这里就省略了把原始坐标转换成世界坐标这一步，理论上模型坐标和射线坐标应该在世界空间内比较
		int point_index = faces->at(i)[0]; 
		glm::vec3 v0 = glm::vec3(vertices->at(point_index)[0], vertices->at(point_index)[1], vertices->at(point_index)[2]);

		point_index = faces->at(i)[1];
		glm::vec3 v1 = glm::vec3(vertices->at(point_index)[0], vertices->at(point_index)[1], vertices->at(point_index)[2]);

		point_index = faces->at(i)[2];
		glm::vec3 v2 = glm::vec3(vertices->at(point_index)[0], vertices->at(point_index)[1], vertices->at(point_index)[2]);

		float t, u, v;
		if (intersect_triangle(camera_pos, orient, v0, v1, v2, &t, &u, &v)) {
			if (t < lastt) { // t越小, 交点距离相机越近
				res = i;
				lastt = t;
			}
		}
	}
	return res;
}

bool PolygonPicker::intersect_triangle(const glm::vec3& orig, const glm::vec3& dir,
	const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, float* t, float* u, float* v) {
	// Determine whether a ray intersect with a triangle
	// Parameters
	// orig: origin of the ray
	// dir: direction of the ray
	// v0, v1, v2: vertices of triangle
	// t(out): weight of the intersection for the ray
	// u(out), v(out): barycentric coordinate of intersection
	// E1
	glm::vec3 E1 = v1 - v0;

	// E2
	glm::vec3 E2 = v2 - v0;

	// P
	//glm::vec3 P = dir.Cross(E2);
	glm::vec3 P = glm::cross(dir, E2);

	// determinant
	//float det = E1.Dot(P);
	float det = glm::dot(E1, P);

	// keep det > 0, modify T accordingly
	glm::vec3 T;
	if (det > 0) {
		T = orig - v0;
	}
	else {
		T = v0 - orig;
		det = -det;
	}

	// If determinant is near zero, ray lies in plane of triangle
	if (det < 0.0001f)
		return false;

	// Calculate u and make sure u <= 1
	//*u = T.Dot(P);
	*u = glm::dot(T, P);
	if (*u < 0.0f || *u > det)
		return false;

	// Q
	//Vector3 Q = T.Cross(E1);
	glm::vec3 Q = glm::cross(T, E1);

	// Calculate v and make sure u + v <= 1
	//*v = dir.Dot(Q);
	*v = glm::dot(dir, Q);
	if (*v < 0.0f || *u + *v > det)
		return false;

	// Calculate t, scale parameters, ray intersects triangle
	//*t = E2.Dot(Q);
	*t = glm::dot(E2, Q);

	float fInvDet = 1.0f / det;
	*t *= fInvDet;
	*u *= fInvDet;
	*v *= fInvDet;

	return true;
}
