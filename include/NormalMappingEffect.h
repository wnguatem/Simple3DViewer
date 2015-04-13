#pragma once

#include "Common.h"
#include "ShaderProgram.h"
#include "VertexDeclarations.h"

namespace Library
{
	class NormalMappingEffect : public ShaderProgram
    {
		//RTTI_DECLARATIONS(NormalMappingEffect, ShaderProgram)

        SHADER_VARIABLE_DECLARATION(WorldViewProjection)
		SHADER_VARIABLE_DECLARATION(World)
		SHADER_VARIABLE_DECLARATION(AmbientColor)
		SHADER_VARIABLE_DECLARATION(LightColor)
		SHADER_VARIABLE_DECLARATION(LightDirection)
		SHADER_VARIABLE_DECLARATION(CameraPosition)
		SHADER_VARIABLE_DECLARATION(SpecularColor)
		SHADER_VARIABLE_DECLARATION(SpecularPower)
		SHADER_VARIABLE_DECLARATION(FogColor)
		SHADER_VARIABLE_DECLARATION(FogStart)
		SHADER_VARIABLE_DECLARATION(FogRange)

    public:
        NormalMappingEffect();
		NormalMappingEffect(const NormalMappingEffect& rhs);
		NormalMappingEffect& operator=(const NormalMappingEffect& rhs);

		virtual void Initialize(GLuint vertexArrayObject) override;
		virtual void CreateVertexBuffer(const Mesh& mesh, GLuint& vertexBuffer) const override;
		void CreateVertexBuffer(VertexPositionTextureNormalTangentBinormal* vertices, unsigned int vertexCount, GLuint& vertexBuffer) const;
		virtual unsigned int VertexSize() const override;

	private:
		enum VertexAttribute
		{
			VertexAttributePosition = 0,
			VertexAttributeTextureCoordinate = 1,
			VertexAttributeNormal = 2,
			VertexAttributeTangent = 3,
			VertexAttributeBinormal = 4
		};
    };
}