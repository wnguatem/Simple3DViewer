#version 440 core

layout (binding = 0) uniform sampler2D ColorTextureSampler;
uniform vec4 AmbientColor;
uniform vec4 LightColor;
uniform vec3 LightPosition;
uniform vec3 CameraPosition;
uniform vec4 SpecularColor;
uniform float SpecularPower;
uniform float SpotLightInnerAngle = 0.75f;
uniform float SpotLightOuterAngle = 0.25f;

in VS_OUTPUT
{
	vec2 TextureCoordinate;
	vec3 Normal;
	vec3 WorldPosition;
	float Attenuation;
	vec3 LightLookAt;
} IN;

out vec4 Color;

void main()
{	
	vec3 lightDirection = normalize(LightPosition - IN.WorldPosition);
	vec3 viewDirection = normalize(CameraPosition - IN.WorldPosition);

	vec3 normal = normalize(IN.Normal);
	float n_dot_l = dot(normal, lightDirection);
	vec3 halfVector = normalize(lightDirection + viewDirection);
	float n_dot_h = dot(normal, halfVector);
	vec3 lightLookAt = normalize(IN.LightLookAt);

	vec4 sampledColor = texture(ColorTextureSampler, IN.TextureCoordinate);
	vec3 ambient = AmbientColor.rgb * sampledColor.rgb;
	vec3 diffuse = clamp(LightColor.rgb * n_dot_l * sampledColor.rgb, 0.0f, 1.0f) * IN.Attenuation;
	
	// specular = N.H^n with gloss map stored in color texture's alpha channel
	vec3 specular = SpecularColor.rgb * min(pow(clamp(n_dot_h, 0.0f, 1.0f), SpecularPower), sampledColor.w) * IN.Attenuation;

	float spotFactor = 0.0f;
	float lightAngle = dot(lightLookAt, lightDirection);	
	if (lightAngle > 0.0f)
	{
    	spotFactor = smoothstep(SpotLightOuterAngle, SpotLightInnerAngle, lightAngle);
	}

	Color.rgb = ambient + (spotFactor * (diffuse + specular));
	Color.a = sampledColor.a;
}