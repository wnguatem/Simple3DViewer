#version 410 core

//layout (binding = 0) uniform samplerCube SkyboxTextureSampler;
uniform samplerCube SkyboxTextureSampler;

in VS_OUTPUT
{
	vec3 TextureCoordinate;
} IN;

out vec4 Color;

void main()
{
	Color = texture(SkyboxTextureSampler, IN.TextureCoordinate);
}