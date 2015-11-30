#pragma once

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
		glm::mat4 mWorldMatrix;
	};
}
