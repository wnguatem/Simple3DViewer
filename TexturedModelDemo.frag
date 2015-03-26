#version 440 core

layout (binding = 0) uniform sampler2D ColorTextureSampler;

in VS_OUTPUT
{
	vec2 TextureCoordinate;
} IN;

out vec4 Color;

void main()
{
	Color = texture(ColorTextureSampler, IN.TextureCoordinate);
}