#include "EnvironmentMappingEffect.h"
#include "GameException.h"
#include "Mesh.h"
#include "ColorHelper.h"

using namespace glm;

namespace Library
{
    //RTTI_DEFINITIONS(EnvironmentMappingEffect)

    EnvironmentMappingEffect::EnvironmentMappingEffect()
        : ShaderProgram(),
		SHADER_VARIABLE_INITIALIZATION(WorldViewProjection), SHADER_VARIABLE_INITIALIZATION(World),
		SHADER_VARIABLE_INITIALIZATION(AmbientColor), SHADER_VARIABLE_INITIALIZATION(EnvironmentColor),
		SHADER_VARIABLE_INITIALIZATION(ReflectionAmount), SHADER_VARIABLE_INITIALIZATION(CameraPosition)
    {
    }

    SHADER_VARIABLE_DEFINITION(EnvironmentMappingEffect, WorldViewProjection)
	SHADER_VARIABLE_DEFINITION(EnvironmentMappingEffect, World)
	SHADER_VARIABLE_DEFINITION(EnvironmentMappingEffect, AmbientColor)
	SHADER_VARIABLE_DEFINITION(EnvironmentMappingEffect, EnvironmentColor)
	SHADER_VARIABLE_DEFINITION(EnvironmentMappingEffect, ReflectionAmount)
	SHADER_VARIABLE_DEFINITION(EnvironmentMappingEffect, CameraPosition)

	void EnvironmentMappingEffect::Initialize(GLuint vertexArrayObject)
    {
        ShaderProgram::Initialize(vertexArrayObject);

        SHADER_VARIABLE_INSTANTIATE(WorldViewProjection)
		SHADER_VARIABLE_INSTANTIATE(World)
		SHADER_VARIABLE_INSTANTIATE(AmbientColor)
		SHADER_VARIABLE_INSTANTIATE(EnvironmentColor)
		SHADER_VARIABLE_INSTANTIATE(ReflectionAmount)
		SHADER_VARIABLE_INSTANTIATE(CameraPosition)

		glVertexAttribPointer(VertexAttributePosition, 4, GL_FLOAT, GL_FALSE, sizeof(VertexPositionTextureNormal), (void*)offsetof(VertexPositionTextureNormal, Position));
		glEnableVertexAttribArray(VertexAttributePosition);

		glVertexAttribPointer(VertexAttributeTextureCoordinate, 2, GL_FLOAT, GL_FALSE, sizeof(VertexPositionTextureNormal), (void*)offsetof(VertexPositionTextureNormal, TextureCoordinates));
		glEnableVertexAttribArray(VertexAttributeTextureCoordinate);

		glVertexAttribPointer(VertexAttributeNormal, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPositionTextureNormal), (void*)offsetof(VertexPositionTextureNormal, Normal));
		glEnableVertexAttribArray(VertexAttributeNormal);
    }

	void EnvironmentMappingEffect::CreateVertexBuffer(const Mesh& mesh, GLuint& vertexBuffer) const
	{
		const std::vector<vec3>& sourceVertices = mesh.Vertices();

		std::vector<VertexPositionTextureNormal> vertices;
		vertices.reserve(sourceVertices.size());

		std::vector<vec3>* textureCoordinates = mesh.TextureCoordinates().at(0);
		assert(textureCoordinates->size() == sourceVertices.size());

		const std::vector<vec3>& normals = mesh.Normals();
		assert(normals.size() == sourceVertices.size());

		for (unsigned int i = 0; i < sourceVertices.size(); i++)
		{
			vec3 position = sourceVertices.at(i);
			vec2 uv = (vec2)textureCoordinates->at(i);
			vec3 normal = normals.at(i);
			vertices.push_back(VertexPositionTextureNormal(vec4(position.x, position.y, position.z, 1.0f), uv, normal));
		}

		CreateVertexBuffer(&vertices[0], vertices.size(), vertexBuffer);
	}

	void EnvironmentMappingEffect::CreateVertexBuffer(VertexPositionTextureNormal* vertices, GLuint vertexCount, GLuint& vertexBuffer) const
	{
		glGenBuffers(1, &vertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
		glBufferData(GL_ARRAY_BUFFER, VertexSize() * vertexCount, &vertices[0], GL_STATIC_DRAW);
	}

    unsigned int EnvironmentMappingEffect::VertexSize() const
    {
        return sizeof(VertexPositionTextureNormal);
    }
}