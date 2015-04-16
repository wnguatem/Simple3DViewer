//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		CubeDemo.h
//\author	William Nguatem
//\note		Copyright (C) 
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26 
//******************************************************************************/


#ifndef _Cube_Demo_H_
#define _Cube_Demo_H_

#include "DrawableGameComponent.h"
#include "ShaderProgram.h"

using namespace Library;

namespace Rendering
{
	class CubeDemo : public DrawableGameComponent
	{
		//RTTI_DECLARATIONS(CubeDemo, DrawableGameComponent)

	public:
		CubeDemo(Game& game, Camera& camera);
		~CubeDemo();

		virtual void Initialize() override;
		virtual void Update(const GameTime& gameTime) override;
		virtual void Draw(const GameTime& gameTime) override;

	private:
		enum VertexAttribute
		{
			VertexAttributePosition = 0,
			VertexAttributeColor = 1
		};

		CubeDemo();
		CubeDemo(const CubeDemo& rhs);
		CubeDemo& operator=(const CubeDemo& rhs);

		static const GLfloat RotationRate;

		ShaderProgram mShaderProgram;
		GLuint mVertexArrayObject;
		GLuint mVertexBuffer;
		GLuint mIndexBuffer;
		GLint mWorldViewProjectionLocation;
		GLuint mColorTexture;
		glm::mat4 mWorldMatrix;

		std::string m_input_data;
		std::wstring mContentFolder;
	};
}
#endif//_CubeDemo_H
