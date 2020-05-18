#version 330 core

out vec3 vertexColor;
out vec3 FragPos;  

uniform vec3 aPos;
uniform vec3 aColor;

void main()
{
    gl_Position = vec4(aPos, 1.0); 
	vertexColor = aColor;
}