#include "NormalMappingEffect.h"
#include "GameException.h"
#include "Mesh.h"
#include "ColorHelper.h"

using namespace glm;

namespace Library
{
    //RTTI_DEFINITIONS(NormalMappingEffect)

    NormalMappingEffect::NormalMappingEffect()
        : ShaderProgram(),
		SHADER_VARIABLE_INITIALIZATION(WorldViewProjection), SHADER_VARIABLE_INITIALIZATION(World),
		SHADER_VARIABLE_INITIALIZATION(AmbientColor), SHADER_VARIABLE_INITIALIZATION(LightColor),
		SHADER_VARIABLE_INITIALIZATION(LightDirection), SHADER_VARIABLE_INITIALIZATION(CameraPosition),
		SHADER_VARIABLE_INITIALIZATION(SpecularColor), SHADER_VARIABLE_INITIALIZATION(SpecularPower),
		SHADER_VARIABLE_INITIALIZATION(FogColor), SHADER_VARIABLE_INITIALIZATION(FogStart),
		SHADER_VARIABLE_INITIALIZATION(FogRange)
    {
    }

    SHADER_VARIABLE_DEFINITION(NormalMappingEffect, WorldViewProjection)
	SHADER_VARIABLE_DEFINITION(NormalMappingEffect, World)
	SHADER_VARIABLE_DEFINITION(NormalMappingEffect, AmbientColor)
	SHADER_VARIABLE_DEFINITION(NormalMappingEffect, LightColor)
	SHADER_VARIABLE_DEFINITION(NormalMappingEffect, LightDirection)
	SHADER_VARIABLE_DEFINITION(NormalMappingEffect, CameraPosition)
	SHADER_VARIABLE_DEFINITION(NormalMappingEffect, SpecularColor)
	SHADER_VARIABLE_DEFINITION(NormalMappingEffect, SpecularPower)
	SHADER_VARIABLE_DEFINITION(NormalMappingEffect, FogColor)
	SHADER_VARIABLE_DEFINITION(NormalMappingEffect, FogStart)
	SHADER_VARIABLE_DEFINITION(NormalMappingEffect, FogRange)

	void NormalMappingEffect::Initialize(GLuint vertexArrayObject)
    {
        ShaderProgram::Initialize(vertexArrayObject);

        SHADER_VARIABLE_INSTANTIATE(WorldViewProjection)
		SHADER_VARIABLE_INSTANTIATE(World)
		SHADER_VARIABLE_INSTANTIATE(AmbientColor)
		SHADER_VARIABLE_INSTANTIATE(LightColor)
		SHADER_VARIABLE_INSTANTIATE(LightDirection)
		SHADER_VARIABLE_INSTANTIATE(CameraPosition)
		SHADER_VARIABLE_INSTANTIATE(SpecularColor)
		SHADER_VARIABLE_INSTANTIATE(SpecularPower)
		SHADER_VARIABLE_INSTANTIATE(FogColor)
		SHADER_VARIABLE_INSTANTIATE(FogStart)
		SHADER_VARIABLE_INSTANTIATE(FogRange)

		glVertexAttribPointer(VertexAttributePosition, 4, GL_FLOAT, GL_FALSE, sizeof(VertexPositionTextureNormalTangentBinormal), (void*)offsetof(VertexPositionTextureNormalTangentBinormal, Position));
		glEnableVertexAttribArray(VertexAttributePosition);

		glVertexAttribPointer(VertexAttributeTextureCoordinate, 2, GL_FLOAT, GL_FALSE, sizeof(VertexPositionTextureNormalTangentBinormal), (void*)offsetof(VertexPositionTextureNormalTangentBinormal, TextureCoordinates));
		glEnableVertexAttribArray(VertexAttributeTextureCoordinate);

		glVertexAttribPointer(VertexAttributeNormal, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPositionTextureNormalTangentBinormal), (void*)offsetof(VertexPositionTextureNormalTangentBinormal, Normal));
		glEnableVertexAttribArray(VertexAttributeNormal);

		glVertexAttribPointer(VertexAttributeTangent, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPositionTextureNormalTangentBinormal), (void*)offsetof(VertexPositionTextureNormalTangentBinormal, Tangent));
		glEnableVertexAttribArray(VertexAttributeTangent);

		glVertexAttribPointer(VertexAttributeBinormal, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPositionTextureNormalTangentBinormal), (void*)offsetof(VertexPositionTextureNormalTangentBinormal, Binormal));
		glEnableVertexAttribArray(VertexAttributeBinormal);
    }

	void NormalMappingEffect::CreateVertexBuffer(const Mesh& mesh, GLuint& vertexBuffer) const
	{
		const std::vector<vec3>& sourceVertices = mesh.Vertices();

		std::vector<VertexPositionTextureNormalTangentBinormal> vertices;
		vertices.reserve(sourceVertices.size());

		std::vector<vec3>* textureCoordinates = mesh.TextureCoordinates().at(0);
		assert(textureCoordinates->size() == sourceVertices.size());

		const std::vector<vec3>& normals = mesh.Normals();
		assert(normals.size() == sourceVertices.size());

		const std::vector<vec3>& tangents = mesh.Tangents();
		assert(tangents.size() == sourceVertices.size());

		const std::vector<vec3>& binormals = mesh.BiNormals();
		assert(binormals.size() == sourceVertices.size());

		for (unsigned int i = 0; i < sourceVertices.size(); i++)
		{
			vec3 position = sourceVertices.at(i);
			vec2 uv = (vec2)textureCoordinates->at(i);
			vec3 normal = normals.at(i);
			vec3 tangent = tangents.at(i);
			vec3 binormal = binormals.at(i);
			vertices.push_back(VertexPositionTextureNormalTangentBinormal(vec4(position.x, position.y, position.z, 1.0f), uv, normal, tangent, binormal));
		}

		CreateVertexBuffer(&vertices[0], vertices.size(), vertexBuffer);
	}

	void NormalMappingEffect::CreateVertexBuffer(VertexPositionTextureNormalTangentBinormal* vertices, GLuint vertexCount, GLuint& vertexBuffer) const
	{
		glGenBuffers(1, &vertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
		glBufferData(GL_ARRAY_BUFFER, VertexSize() * vertexCount, &vertices[0], GL_STATIC_DRAW);
	}

    unsigned int NormalMappingEffect::VertexSize() const
    {
        return sizeof(VertexPositionTextureNormalTangentBinormal);
    }
}