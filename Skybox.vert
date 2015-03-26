#version 440 core

uniform mat4 WorldViewProjection;

layout (location = 0) in vec4 Position;

out VS_OUTPUT
{
	vec3 TextureCoordinate;
} OUT;

void main()
{	
	gl_Position = WorldViewProjection * Position;
	OUT.TextureCoordinate = Position.xyz;
}