#include "opengl_window.h"

#include <display/polygon_picker.h>

thread_local ImGuiContext* MyImGuiTLS = nullptr; // imgui多线程

// 全局窗口对象总数
int OpenGLWindow::global_window_num = 0;
// 设置窗口大小
unsigned OpenGLWindow::SCR_WIDTH = 800.0f;
unsigned OpenGLWindow::SCR_HEIGHT = 800.0f; // 窗口长等于宽时，射线检测结果才正确
// 相机
Camera OpenGLWindow::camera(glm::vec3(0.0f, 10.0f, 50.0f));
float OpenGLWindow::lastX = OpenGLWindow::SCR_WIDTH / 2.0f;
float OpenGLWindow::lastY = OpenGLWindow::SCR_HEIGHT / 2.0f;
float OpenGLWindow::xpos = OpenGLWindow::SCR_WIDTH / 2.0f;
float OpenGLWindow::ypos = OpenGLWindow::SCR_HEIGHT / 2.0f;
bool OpenGLWindow::firstMouse = true;
bool OpenGLWindow::mouse_right_down = false;
// 帧间间隔
thread_local float OpenGLWindow::deltaTime = 0.0f;
thread_local float OpenGLWindow::lastFrame = 0.0f;
// 坐标变换
glm::mat4 OpenGLWindow::model;
glm::mat4 OpenGLWindow::view;
glm::mat4 OpenGLWindow::projection;
// 工具
 thread_local PolygonPicker* OpenGLWindow::polygon_picker = nullptr;
 thread_local int OpenGLWindow::picked_face = -1;
 thread_local vector<vector<int>> OpenGLWindow::picked_patch_info;
 // 锁
std::mutex OpenGLWindow::mut;

OpenGLWindow::OpenGLWindow(bool _recovered_mesh, const std::string& _title, const std::string& _vertex_shader, const std::string& _fragment_shader, const std::vector<float>* _vertex_data, const std::vector<float>* _color_data):
	recovered_mesh(_recovered_mesh),
	title(_title),
	vertex_data(_vertex_data),
	color_data(_color_data)
{
	{
		std::lock_guard<std::mutex> guard(mut);
		if (global_window_num == 0) {
			glfwInit();
		}
	
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
		glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE); // 禁止改变窗口大小

		window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, title.c_str(), NULL, NULL);
		if (window == NULL) {
			std::cout << "创建GLFW窗口失败!" << std::endl;
			glfwTerminate();
			return;
		}
	
		glfwMakeContextCurrent(window);
		glfwSetCursorPosCallback(window, mouse_callback);
		glfwSetScrollCallback(window, scroll_callback);
		glfwSetMouseButtonCallback(window, mouse_button_callback);
		glfwSwapInterval(1); // Enable vsync

		if (recovered_mesh) glfwSetWindowPos(window, 200.0f + SCR_WIDTH, 100.0f);
		else glfwSetWindowPos(window, 100.0f, 100.0f);

		if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) { // GLAD是用来管理OpenGL的函数指针的，所以在调用任何OpenGL的函数之前我们需要初始化GLAD
			std::cout << "Failed to initialize GLAD" << std::endl;
			return;
		}

		glEnable(GL_DEPTH_TEST);
		glDepthFunc(GL_LESS);

		//创建并绑定ImGui
		const char* glsl_version = "#version 130";
		IMGUI_CHECKVERSION();
		ImGui::CreateContext();
		ImGuiIO& io = ImGui::GetIO(); (void)io;
		ImGui::StyleColorsDark();
		ImGui_ImplGlfw_InitForOpenGL(window, true);
		ImGui_ImplOpenGL3_Init(glsl_version);

		ourShader = new Shader(_vertex_shader.c_str(), _fragment_shader.c_str()); // 创建shader需要调用gl函数，所以必须放在glad加载函数之后

		++global_window_num;
	}
}

OpenGLWindow::~OpenGLWindow() {
	std::lock_guard<std::mutex> guard(mut);
	--global_window_num;

	glDeleteVertexArrays(1, &vao_mesh);
	glDeleteBuffers(1, &vbo_vertex);
	glDeleteBuffers(1, &vbo_color);

	if (window != nullptr) {
		glfwDestroyWindow(window);
		window = nullptr;
	}
	if (global_window_num == 0) {
		glfwTerminate();
	}
}

void OpenGLWindow::show() {
	ImGuiWindowFlags window_flags = 0;
	window_flags |= ImGuiWindowFlags_NoBackground;
	window_flags |= ImGuiTreeNodeFlags_DefaultOpen;
	//window_flags |= ImGuiWindowFlags_NoTitleBar;
	//window_flags |= ImGuiWindowFlags_MenuBar;
	while (!glfwWindowShouldClose(window)) {
		float currentFrame = static_cast<float>(glfwGetTime());
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		processInput(window);

		glClearColor(0.24f, 0.24f, 0.24f, 1.0f); // (0.2f, 0.3f, 0.3f, 1.0f)
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// 创建ImGui
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		ImGui::Begin("Imgui Panel", &imgui, window_flags);
		ImGui::Text("This window shows %s mesh.", title.c_str());
		ImGui::Text("Average %.3f ms/frame (%.1f FPS).", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

		if (ImGui::CollapsingHeader("Help")) {
			ImGui::Text("HOW TO MOVE");
			ImGui::BulletText("Hold the right mouse button(RB) first."); // BulletText：前面有无序编号的圆点
			ImGui::BulletText("RB + W: move forward.");
			ImGui::BulletText("RB + S: move back.");
			ImGui::BulletText("RB + A: move left.");
			ImGui::BulletText("RB + D: move right.");
			ImGui::BulletText("RB + moving mouse: look around.");
			ImGui::Text("HOW TO SELECT");
			ImGui::BulletText("Click on a polygon in the mesh.");
		}
		
		if (ImGui::CollapsingHeader("Control", ImGuiTreeNodeFlags_DefaultOpen)) {
			ImGui::SliderFloat("camera zoom", &camera.Zoom, 10.0f, 90.0f, "zoom = %.3f");
			ImGui::SliderFloat("camera speed", &camera.MovementSpeed, 1.0f, 100.0f, "speed = %.3f");
			ImGui::Checkbox("draw rays(while clicking)", &show_ray);
		}

		if (ImGui::CollapsingHeader("Detail", ImGuiTreeNodeFlags_DefaultOpen)) {
			ImGui::SetNextItemOpen(true);
			if (ImGui::TreeNode("MESH INFORMATION")) {
				ImGui::Text("total vertices: %d", polygon_picker->get_vertices_num());
				ImGui::Text("total faces/polygons: %d", polygon_picker->get_faces_num());
				ImGui::TreePop(); // 这句对于TreeNode来说不能少
			}

			ImGui::SetNextItemOpen(true);
			if (ImGui::TreeNode("PATCH/COMPRESSION INFORMATION")) {
				ImGui::Text("total patches: %d", polygon_picker->get_patch_num());
				ImGui::Text("average size of patches: %.3f", polygon_picker->get_vertices_num() * 1.0f / polygon_picker->get_patch_num());
				ImGui::Text("length of patch feature: %d", polygon_picker->get_feature_len());
				ImGui::Text("operators in dictionary: %d", polygon_picker->get_atoms());
				ImGui::TreePop();
			}

			ImGui::SetNextItemOpen(true);
			if (ImGui::TreeNode("POLYGON INFORMATION")) {
				if (picked_face == -1) ImGui::Text("no polygon selected");
				else {
					ImGui::Text("selected polygon: %d", picked_face);
					if (picked_patch_info.size() > 0) {
						ImGui::Text("selected polygon covers following patches:");
						for (int i = 0; i < picked_patch_info.size(); ++i) {
							ImGui::BulletText("patch: %d, size: %d", picked_patch_info[i][0], picked_patch_info[i][1]);
						}
					}
					else ImGui::Text("selected polygon doesn't cover any patches");
				}
				ImGui::TreePop();
			}
		}
		ImGui::End();

		ourShader->use();
		projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 250.0f);
		ourShader->setMat4("projection", projection);
		view = camera.GetViewMatrix();
		ourShader->setMat4("view", view);
		model = glm::mat4(1.0f);
		ourShader->setMat4("model", model);

		glBindVertexArray(vao_mesh);
		glDrawArrays(GL_TRIANGLES, 0, vertex_data->size() / 3); // 注意调整顶点总数！！

		// 绘制射线
		if (show_ray) {
			polygon_picker->draw_ray(model, view, projection);
		}

		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	if (window != nullptr) {
		glfwDestroyWindow(window);
		window = nullptr;
	}
}

void OpenGLWindow::mouse_callback(GLFWwindow* window, double xposIn, double yposIn) { // 只捕获鼠标移动，不捕获鼠标点击
	xpos = float(xposIn); // 刚打开窗口的时候，如果鼠标指针就在窗口内，也会调这个函数，所以初始位置是正确的
	ypos = float(yposIn);
	if (!mouse_right_down) return;
	if (firstMouse) {
		lastX = xpos;
		lastY = ypos;
		firstMouse = false;
	}
	float xoffset = xpos - lastX;
	float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top
	lastX = xpos;
	lastY = ypos;
	camera.ProcessMouseMovement(xoffset, yoffset);
}

void OpenGLWindow::scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
	camera.ProcessMouseScroll(float(yoffset));
}

void OpenGLWindow::mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
	if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
		mouse_right_down = true;
	}
	else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE) {
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
		mouse_right_down = false;
		firstMouse = true;
	}
	else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		glm::vec3 position = camera.Position;

		picked_face = polygon_picker->select_triangle(xpos, ypos, SCR_WIDTH, SCR_HEIGHT, projection, view, position);

		vector<int> patch_faces;
		picked_patch_info = polygon_picker->get_patch_info(picked_face, &patch_faces);
		int picked_patch = picked_patch_info.size() == 1 ? picked_patch_info[0][0] : -1;
		polygon_picker->update_color(picked_face, picked_patch, &patch_faces);
	}
}

void OpenGLWindow::processInput(GLFWwindow* window) {
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
		glfwSetWindowShouldClose(window, true);
	}
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS && mouse_right_down) {
		camera.ProcessKeyboard(FORWARD, deltaTime);
	}
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS && mouse_right_down) {
		camera.ProcessKeyboard(BACKWARD, deltaTime);
	}
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS && mouse_right_down) {
		camera.ProcessKeyboard(LEFT, deltaTime);
	}
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS && mouse_right_down) {
		camera.ProcessKeyboard(RIGHT, deltaTime);
	}
}

void OpenGLWindow::bind_buffer() {
	glGenVertexArrays(1, &vao_mesh);

	glGenBuffers(1, &vbo_vertex);
	glBindVertexArray(vao_mesh);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_vertex);
	glBufferData(GL_ARRAY_BUFFER, vertex_data->size() * sizeof(float), &vertex_data->at(0), GL_STATIC_DRAW);
	glVertexAttribPointer( // 调用之前先glBindBuffer，顶点属性位置（第一个参数）就能找到对应的VBO
		0,        // attribute 0. No particular reason for 0, but must match the layout in the shader.
		3,        // size // 当前顶点属性由多少个float组成，如果是坐标的话就是3，如果是uv坐标的话就是2
		GL_FLOAT, // type
		GL_FALSE, // normalized?
		3 * sizeof(float),        // stride 
		(void*)0 // array buffer offset
	);
	glEnableVertexAttribArray(0);

	glGenBuffers(1, &vbo_color);
	glBindVertexArray(vao_mesh);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_color);
	glBufferData(GL_ARRAY_BUFFER, color_data->size() * sizeof(float), &color_data->at(0), GL_STATIC_DRAW);
	glVertexAttribPointer( // 调用之前先glBindBuffer，顶点属性位置（第一个参数）就能找到对应的VBO
		1,        // attribute 0. No particular reason for 0, but must match the layout in the shader.
		3,        // size // 当前顶点属性由多少个float组成，如果是坐标的话就是3，如果是uv坐标的话就是2
		GL_FLOAT, // type
		GL_FALSE, // normalized?
		3 * sizeof(float),        // stride 
		(void*)0 // array buffer offset
	);
	glEnableVertexAttribArray(1);
}