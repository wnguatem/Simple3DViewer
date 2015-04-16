#version 410 core

uniform mat4 WorldViewProjection;
uniform mat4 World;
uniform vec3 CameraPosition;

layout (location = 0) in vec4 Position;
layout (location = 1) in vec2 TextureCoordinate;
layout (location = 2) in vec3 Normal;

out VS_OUTPUT
{
	vec2 TextureCoordinate;
	vec3 ReflectionVector;
} OUT;

void main()
{	
	gl_Position = WorldViewProjection * Position;
	OUT.TextureCoordinate = TextureCoordinate;
	
	vec3 worldPosition = (World * Position).xyz;
	vec3 incident = normalize(worldPosition - CameraPosition);
	vec3 normal = (World * vec4(Normal, 0.0f)).xyz;
	
	// Reflection Vector for cube map: R = I - 2 * dot(N, I) * N
	OUT.ReflectionVector = reflect(incident, normal);
}