#version 410 core

//layout (binding = 0) uniform sampler2D ColorTextureSampler;
uniform sampler2D ColorTextureSampler;
uniform vec4 AmbientColor;

in VS_OUTPUT
{
	vec2 TextureCoordinate;
} IN;

out vec4 Color;

void main()
{
	vec4 sampledColor = texture(ColorTextureSampler, IN.TextureCoordinate);
	Color.rgb = sampledColor.rgb * AmbientColor.rgb;
	Color.a = sampledColor.a;
	//Color = sampledColor;
//Color = texture(ColorTextureSampler, IN.TextureCoordinate);
}