//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		Grid.h
//\author	William Nguatem
//\note		Copyright (C) 
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26 
//******************************************************************************/


#ifndef _Grid_H_
#define _Grid_H_

#include "Common.h"
#include "DrawableGameComponent.h"
#include "BasicEffect.h"

namespace Library
{
	class Grid : public DrawableGameComponent
	{
		//RTTI_DECLARATIONS(Grid, DrawableGameComponent)

	public:
		Grid(Game& game, Camera& camera);
		Grid(Game& game, Camera& camera, GLuint size, GLuint scale, const glm::vec4& color);
		~Grid();

		const glm::vec3& Position() const;
		const glm::vec4& Color() const;
		const GLuint Size() const;
		const GLuint Scale() const;

		void SetPosition(const glm::vec3& position);
		void SetPosition(float x, float y, float z);
		void SetColor(const glm::vec4& color);
		void SetSize(GLuint size);
		void SetScale(GLuint scale);

		virtual void Initialize() override;
		virtual void Draw(const GameTime& gameTime) override;

	private:
		Grid();
		Grid(const Grid& rhs);
		Grid& operator=(const Grid& rhs);
		
		void InitializeGrid();

		static const GLuint DefaultSize;
		static const GLuint DefaultScale;
		static const glm::vec4 DefaultColor;

		BasicEffect mShaderProgram;
		GLuint mVertexArrayObject;
		GLuint mVertexBuffer;
	
		glm::vec3 mPosition;
		GLuint mSize;
		GLuint mScale;
		glm::vec4 mColor;
		glm::mat4 mWorldMatrix;
		unsigned int mVertexCount;
		std::wstring mContentFolder;
	};
}
#endif//_Grid_H_