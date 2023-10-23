#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <learnopengl/shader.h>
#include <learnopengl/camera.h>

#include <display/polygon_picker.h>

#include <iostream>
#include <mutex>

class OpenGLWindow {
public:
	OpenGLWindow(bool _recovered_mesh, const std::string& _title, const std::string&_vertex_shader, const std::string& _fragment_shader, 
		const std::vector<float>* _vertex_data, const std::vector<float>* _color_data);
	~OpenGLWindow();

	// 全局窗口对象总数
	static int global_window_num;
	// 设置窗口大小
	static unsigned SCR_WIDTH;
	static unsigned SCR_HEIGHT;
	// 相机
	static Camera camera;
	static float lastX;
	static float lastY;
	static float xpos;
	static float ypos;
	static bool firstMouse;
	static bool mouse_right_down;
	// 帧间间隔
	static thread_local float deltaTime;
	static thread_local float lastFrame;
	// 坐标变换
	static glm::mat4 model;
	static glm::mat4 view;
	static glm::mat4 projection;
	// 工具
	static thread_local PolygonPicker* polygon_picker;
	static thread_local int picked_face;
	static thread_local vector<vector<int>> picked_patch_info;
	// VAO和VBO
	unsigned int vao_mesh;
	unsigned int vbo_vertex;
	unsigned int vbo_color;
	// 锁
	static std::mutex mut;

	// 渲染循环
	void show();
	// 绑定VAO和VBO
	void bind_buffer();

private:
	// opengl回调函数
	static void mouse_callback(GLFWwindow* window, double xposIn, double yposIn); 
	static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
	static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
	// 处理键盘输入
	void processInput(GLFWwindow* window);
	
	// 窗口
	const bool recovered_mesh;
	const std::string title;
	GLFWwindow* window;
	// 数据
	const std::vector<float>* vertex_data; // 传入shader的坐标数组
	const std::vector<float>* color_data; // 传入shader的颜色数组
	// 显示控制
	bool imgui = true;
	bool show_ray = false;
	// 着色器
	Shader* ourShader;
};
