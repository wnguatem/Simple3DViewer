#pragma once

#include "Common.h"
#include "ShaderProgram.h"
#include "VertexDeclarations.h"

namespace Library
{
	class EnvironmentMappingEffect : public ShaderProgram
    {
		//RTTI_DECLARATIONS(EnvironmentMappingEffect, ShaderProgram)

        SHADER_VARIABLE_DECLARATION(WorldViewProjection)
		SHADER_VARIABLE_DECLARATION(World)
		SHADER_VARIABLE_DECLARATION(AmbientColor)
		SHADER_VARIABLE_DECLARATION(EnvironmentColor)
		SHADER_VARIABLE_DECLARATION(ReflectionAmount)
		SHADER_VARIABLE_DECLARATION(CameraPosition)

    public:
        EnvironmentMappingEffect();
		EnvironmentMappingEffect(const EnvironmentMappingEffect& rhs);
		EnvironmentMappingEffect& operator=(const EnvironmentMappingEffect& rhs);

		virtual void Initialize(GLuint vertexArrayObject) override;
		virtual void CreateVertexBuffer(const Mesh& mesh, GLuint& vertexBuffer) const override;
		void CreateVertexBuffer(VertexPositionTextureNormal* vertices, unsigned int vertexCount, GLuint& vertexBuffer) const;
		virtual unsigned int VertexSize() const override;

	private:
		enum VertexAttribute
		{
			VertexAttributePosition = 0,
			VertexAttributeTextureCoordinate = 1,
			VertexAttributeNormal = 2
		};
    };
}