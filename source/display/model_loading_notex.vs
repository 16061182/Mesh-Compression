#version 330 core
layout (location = 0) in vec3 aPos;
// layout (location = 1) in vec3 aNormal;
// layout (location = 2) in vec2 aTexCoords;
layout (location = 1) in vec3 aColor;

// out vec2 TexCoords;
out vec3 color;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
// uniform bool highlight = false; // set default
// uniform vec3 highlight_color = vec3(1.0, 1.0, 0.0);

void main()
{
    // TexCoords = aTexCoords;    
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    color = aColor;
}