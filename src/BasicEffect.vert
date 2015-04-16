#version 410 core

uniform mat4 WorldViewProjection;

layout (location = 0) in vec4 Position;
layout (location = 1) in vec4 Color;

out VS_OUTPUT
{
	vec4 Color;
} OUT;

void main()
{	
	gl_Position = WorldViewProjection * Position;
	OUT.Color = Color;
}