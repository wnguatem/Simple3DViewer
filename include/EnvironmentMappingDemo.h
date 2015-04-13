#pragma once

#include "DrawableGameComponent.h"
#include "EnvironmentMappingEffect.h"

namespace Library
{
	class Light;
}

using namespace Library;

namespace Rendering
{
	class EnvironmentMappingDemo : public DrawableGameComponent
	{
		//RTTI_DECLARATIONS(EnvironmentMappingDemo, DrawableGameComponent)

	public:		
		EnvironmentMappingDemo(Game& game, Camera& camera);
		~EnvironmentMappingDemo();

		virtual void Initialize() override;
		virtual void Update(const GameTime& gameTime) override;
		virtual void Draw(const GameTime& gameTime) override;

	private:
		EnvironmentMappingDemo();
		EnvironmentMappingDemo(const EnvironmentMappingDemo& rhs);
		EnvironmentMappingDemo& operator=(const EnvironmentMappingDemo& rhs);

		void UpdateAmbientLight(const GameTime& gameTime);
		void UpdateReflectionAmount(const GameTime& gameTime);
		
		EnvironmentMappingEffect mShaderProgram;
		GLuint mVertexArrayObject;
		GLuint mVertexBuffer;
		GLuint mIndexBuffer;
		glm::mat4 mWorldMatrix;
		GLuint mIndexCount;
		GLuint mColorTexture;
		GLuint mEnvironmentMap;
		Light* mAmbientLight;
		glm::vec4 mEnvironmentColor;
		float mReflectionAmount;
		GLuint mColorTextureSampler;
		GLuint mEnvironmentMapSampler;
	};
}
