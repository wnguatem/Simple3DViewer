#version 440 core

uniform mat4 WorldViewProjection;
uniform mat4 World;
uniform vec3 LightPosition;
uniform float LightRadius = 10.0f;
uniform vec3 LightLookAt;

layout (location = 0) in vec4 Position;
layout (location = 1) in vec2 TextureCoordinate;
layout (location = 2) in vec3 Normal;

out VS_OUTPUT
{
	vec2 TextureCoordinate;
	vec3 Normal;
	vec3 WorldPosition;
	float Attenuation;
	vec3 LightLookAt;
} OUT;

void main()
{	
	gl_Position = WorldViewProjection * Position;
	OUT.TextureCoordinate = TextureCoordinate;
	OUT.Normal = (World * vec4(Normal, 0.0f)).xyz;
	OUT.WorldPosition = (World * Position).xyz;

	vec3 lightDirection = LightPosition - OUT.WorldPosition;
	OUT.Attenuation = clamp(1.0f - (length(lightDirection) / LightRadius), 0.0f, 1.0f);
	OUT.LightLookAt = -LightLookAt;
}