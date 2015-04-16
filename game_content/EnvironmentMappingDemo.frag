#version 410 core

//layout (binding = 0) uniform samplerCube EnvironmentMapSampler;
uniform samplerCube EnvironmentMapSampler;
//layout (binding = 1) uniform sampler2D ColorTextureSampler;
uniform sampler2D ColorTextureSampler;

uniform vec4 AmbientColor;
uniform vec4 EnvironmentColor;
uniform float ReflectionAmount;

in VS_OUTPUT
{
	vec2 TextureCoordinate;
	vec3 ReflectionVector;
} IN;

out vec4 Color;

void main()
{	
	vec4 sampledColor = texture(ColorTextureSampler, IN.TextureCoordinate);
	vec3 ambient = AmbientColor.rgb * sampledColor.rgb;
	vec3 environment = texture(EnvironmentMapSampler, IN.ReflectionVector).rgb;
	vec3 reflection = EnvironmentColor.rgb * environment;

	Color.rgb = mix(ambient, reflection, ReflectionAmount);
	Color.a = sampledColor.a;
}