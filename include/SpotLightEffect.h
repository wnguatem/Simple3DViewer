#pragma once

#include "Common.h"
#include "ShaderProgram.h"
#include "VertexDeclarations.h"

namespace Library
{
	class SpotLightEffect : public ShaderProgram
    {
		//RTTI_DECLARATIONS(SpotLightEffect, ShaderProgram)

        SHADER_VARIABLE_DECLARATION(WorldViewProjection)
		SHADER_VARIABLE_DECLARATION(World)
		SHADER_VARIABLE_DECLARATION(AmbientColor)
		SHADER_VARIABLE_DECLARATION(LightColor)
		SHADER_VARIABLE_DECLARATION(LightPosition)
		SHADER_VARIABLE_DECLARATION(LightRadius)
		SHADER_VARIABLE_DECLARATION(LightLookAt)
		SHADER_VARIABLE_DECLARATION(CameraPosition)
		SHADER_VARIABLE_DECLARATION(SpecularColor)
		SHADER_VARIABLE_DECLARATION(SpecularPower)
		SHADER_VARIABLE_DECLARATION(SpotLightInnerAngle)
		SHADER_VARIABLE_DECLARATION(SpotLightOuterAngle)

    public:
        SpotLightEffect();
		SpotLightEffect(const SpotLightEffect& rhs);
		SpotLightEffect& operator=(const SpotLightEffect& rhs);

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