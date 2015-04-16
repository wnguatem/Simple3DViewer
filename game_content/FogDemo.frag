#version 410 core

//layout (binding = 0) uniform sampler2D ColorTextureSampler;
uniform sampler2D ColorTextureSampler;
uniform vec4 AmbientColor;
uniform vec4 LightColor;
uniform vec3 CameraPosition;
uniform vec4 SpecularColor;
uniform float SpecularPower;
uniform vec4 FogColor;


in VS_OUTPUT
{
	vec2 TextureCoordinate;
	vec3 Normal;
	vec3 LightDirection;
	vec3 WorldPosition;
	float FogAmount;
} IN;

out vec4 Color;

void main()
{
	if (IN.FogAmount == 1.0f)
	{
		Color = vec4(FogColor.rgb, 1.0f);
		return;
	}

    vec3 lightDirection = normalize(IN.LightDirection);
	vec3 viewDirection = normalize(CameraPosition - IN.WorldPosition);

	vec3 normal = normalize(IN.Normal);
	float n_dot_l = dot(lightDirection, normal);
	vec3 halfVector = normalize(lightDirection + viewDirection);
	float n_dot_h = dot(normal, halfVector);

	vec4 sampledColor = texture(ColorTextureSampler, IN.TextureCoordinate);
	vec3 ambient = AmbientColor.rgb * sampledColor.rgb;
	vec3 diffuse = clamp(LightColor.rgb * n_dot_l * sampledColor.rgb, 0.0f, 1.0f);	
	
	// specular = N.H^n with gloss map stored in color texture's alpha channel
	vec3 specular = SpecularColor.rgb * min(pow(clamp(n_dot_h, 0.0f, 1.0f), SpecularPower), sampledColor.w);

	Color.rgb = ambient + diffuse + specular;
	Color.a = sampledColor.a;

	Color.rgb = mix(Color.rgb, FogColor.rgb, IN.FogAmount);
}