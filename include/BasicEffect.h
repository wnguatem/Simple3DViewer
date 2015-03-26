//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		BasicEffect.h
//\author	William Nguatem
//\note		Copyright (C) 
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26 
//******************************************************************************/


#ifndef _Basic_Effect_H_
#define _Basic_Effect_H_

#include "Common.h"
#include "ShaderProgram.h"
#include "VertexDeclarations.h"

namespace Library
{
	class BasicEffect : public ShaderProgram
    {
		////RTTI_DECLARATIONS(BasicEffect, ShaderProgram)

        SHADER_VARIABLE_DECLARATION(WorldViewProjection)

    public:
        BasicEffect();
		BasicEffect(const BasicEffect& rhs);
		BasicEffect& operator=(const BasicEffect& rhs);

		virtual void Initialize(GLuint vertexArrayObject) override;
		virtual void CreateVertexBuffer(const Mesh& mesh, GLuint& vertexBuffer) const override;
		void CreateVertexBuffer(VertexPositionColor* vertices, unsigned int vertexCount, GLuint& vertexBuffer) const;
		virtual unsigned int VertexSize() const override;

	private:
		enum VertexAttribute
		{
			VertexAttributePosition = 0,
			VertexAttributeColor = 1
		};
    };
}
#endif//_Basic_Effect_H_