#version 440 core

uniform mat4 WorldViewProjection;

layout (location = 0) in vec4 Position;
layout (location = 1) in vec2 TextureCoordinate;

out VS_OUTPUT
{
	vec2 TextureCoordinate;
} OUT;

void main()
{	
	gl_Position = WorldViewProjection * Position;
	OUT.TextureCoordinate = TextureCoordinate;
}