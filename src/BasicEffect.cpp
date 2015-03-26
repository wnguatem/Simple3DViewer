#include "BasicEffect.h"
#include "GameException.h"
#include "Mesh.h"
#include "ColorHelper.h"

using namespace glm;

namespace Library
{
    //RTTI_DEFINITIONS(BasicEffect)	

    BasicEffect::BasicEffect()
        : ShaderProgram(),
          SHADER_VARIABLE_INITIALIZATION(WorldViewProjection)
    {
    }

    SHADER_VARIABLE_DEFINITION(BasicEffect, WorldViewProjection)

    void BasicEffect::Initialize(GLuint vertexArrayObject)
    {
        ShaderProgram::Initialize(vertexArrayObject);

        SHADER_VARIABLE_INSTANTIATE(WorldViewProjection)

		glVertexAttribPointer(VertexAttributePosition, 4, GL_FLOAT, GL_FALSE, sizeof(VertexPositionColor), (void*)offsetof(VertexPositionColor, Position));
		glEnableVertexAttribArray(VertexAttributePosition);

		glVertexAttribPointer(VertexAttributeColor, 4, GL_FLOAT, GL_FALSE, sizeof(VertexPositionColor), (void*)offsetof(VertexPositionColor, Color));
		glEnableVertexAttribArray(VertexAttributeColor);
    }

	void BasicEffect::CreateVertexBuffer(const Mesh& mesh, GLuint& vertexBuffer) const
	{
		const std::vector<vec3>& sourceVertices = mesh.Vertices();

		std::vector<VertexPositionColor> vertices;
		vertices.reserve(sourceVertices.size());
		if (mesh.VertexColors().size() > 0)
		{
			std::vector<vec4>* vertexColors = mesh.VertexColors().at(0);
			assert(vertexColors->size() == sourceVertices.size());

			for (size_t i = 0; i < sourceVertices.size(); i++)
			{
				vec3 position = sourceVertices.at(i);
				vec4 color = vertexColors->at(i);
				vertices.push_back(VertexPositionColor(vec4(position.x, position.y, position.z, 1.0f), color));
			}
		}
		else
		{
			for (size_t i = 0; i < sourceVertices.size(); i++)
			{
				vec3 position = sourceVertices.at(i);
				vertices.push_back(VertexPositionColor(vec4(position.x, position.y, position.z, 1.0f), ColorHelper::White));
			}
		}

		CreateVertexBuffer(&vertices[0], vertices.size(), vertexBuffer);
	}

	void BasicEffect::CreateVertexBuffer(VertexPositionColor* vertices, GLuint vertexCount, GLuint& vertexBuffer) const
	{
		glGenBuffers(1, &vertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
		glBufferData(GL_ARRAY_BUFFER, VertexSize() * vertexCount, &vertices[0], GL_STATIC_DRAW);
	}

    unsigned int BasicEffect::VertexSize() const
    {
        return sizeof(VertexPositionColor);
    }
}