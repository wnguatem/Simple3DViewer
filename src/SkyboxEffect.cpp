#include "SkyboxEffect.h"
#include "GameException.h"
#include "Mesh.h"
#include "ColorHelper.h"

using namespace glm;

namespace Library
{
    //RTTI_DEFINITIONS(SkyboxEffect)	

    SkyboxEffect::SkyboxEffect()
        : ShaderProgram(),
          SHADER_VARIABLE_INITIALIZATION(WorldViewProjection)
    {
    }

    SHADER_VARIABLE_DEFINITION(SkyboxEffect, WorldViewProjection)

    void SkyboxEffect::Initialize(GLuint vertexArrayObject)
    {
        ShaderProgram::Initialize(vertexArrayObject);

        SHADER_VARIABLE_INSTANTIATE(WorldViewProjection)

		glVertexAttribPointer(VertexAttributePosition, 4, GL_FLOAT, GL_FALSE, sizeof(VertexPosition), (void*)offsetof(VertexPosition, Position));
		glEnableVertexAttribArray(VertexAttributePosition);
    }

	void SkyboxEffect::CreateVertexBuffer(const Mesh& mesh, GLuint& vertexBuffer) const
	{
		const std::vector<vec3>& sourceVertices = mesh.Vertices();

		std::vector<VertexPosition> vertices;
		vertices.reserve(sourceVertices.size());
		for (size_t i = 0; i < sourceVertices.size(); i++)
		{
			vec3 position = sourceVertices.at(i);
			vertices.push_back(VertexPosition(vec4(position.x, position.y, position.z, 1.0f)));
		}

		CreateVertexBuffer(&vertices[0], vertices.size(), vertexBuffer);
	}

	void SkyboxEffect::CreateVertexBuffer(VertexPosition* vertices, GLuint vertexCount, GLuint& vertexBuffer) const
	{
		glGenBuffers(1, &vertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
		glBufferData(GL_ARRAY_BUFFER, VertexSize() * vertexCount, &vertices[0], GL_STATIC_DRAW);
	}

    unsigned int SkyboxEffect::VertexSize() const
    {
        return sizeof(VertexPosition);
    }
}