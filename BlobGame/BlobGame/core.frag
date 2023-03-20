#version 430 core

const int hull_size = 200;

in vec3 ourColor;

out vec4 color;

uniform vec2 hull[hull_size];

void main()
{
	color = vec4(ourColor, 1.0f);
}