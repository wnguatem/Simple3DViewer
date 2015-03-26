//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		SkyboxEffect.h
//\author	William Nguatem
//\note		Copyright (C) 
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26 
//******************************************************************************/


#ifndef _Skybox_Effect_H_
#define _Skybox_Effect_H_

#include "Common.h"
#include "ShaderProgram.h"
#include "VertexDeclarations.h"

namespace Library
{
	class SkyboxEffect : public ShaderProgram
    {
		//RTTI_DECLARATIONS(SkyboxEffect, ShaderProgram)

        SHADER_VARIABLE_DECLARATION(WorldViewProjection)

    public:
        SkyboxEffect();
		SkyboxEffect(const SkyboxEffect& rhs);
		SkyboxEffect& operator=(const SkyboxEffect& rhs);

		virtual void Initialize(GLuint vertexArrayObject) override;
		virtual void CreateVertexBuffer(const Mesh& mesh, GLuint& vertexBuffer) const override;
		void CreateVertexBuffer(VertexPosition* vertices, unsigned int vertexCount, GLuint& vertexBuffer) const;
		virtual unsigned int VertexSize() const override;

	private:
		enum VertexAttribute
		{
			VertexAttributePosition = 0,
		};
    };
}
#endif//_Skybox_Effect_H_