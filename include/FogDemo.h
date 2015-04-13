#pragma once

#include "DrawableGameComponent.h"
#include "FogEffect.h"

namespace Library
{
	class Light;
	class DirectionalLight;
	class ProxyModel;
}

using namespace Library;

namespace Rendering
{
	class FogDemo : public DrawableGameComponent
	{
		//RTTI_DECLARATIONS(FogDemo, DrawableGameComponent)

	public:
		FogDemo(Game& game, Camera& camera);
		~FogDemo();

		virtual void Initialize() override;
		virtual void Update(const GameTime& gameTime) override;
		virtual void Draw(const GameTime& gameTime) override;

	private:
		FogDemo();
		FogDemo(const FogDemo& rhs);
		FogDemo& operator=(const FogDemo& rhs);

		void UpdateAmbientLight(const GameTime& gameTime);
		void UpdateDirectionalLight(const GameTime& gameTime);
		void UpdateSpecularLight(const GameTime& gameTime);

		static const glm::vec2 LightRotationRate;
		static const float LightModulationRate;

		FogEffect mShaderProgram;
		GLuint mVertexArrayObject;
		GLuint mVertexBuffer;
		GLuint mIndexBuffer;
		glm::mat4 mWorldMatrix;
		GLuint mIndexCount;
		GLuint mColorTexture;
		Light* mAmbientLight;
		DirectionalLight* mDirectionalLight;
		glm::vec4 mSpecularColor;
		float mSpecularPower;
		glm::vec4 mFogColor;
		float mFogStart;
		float mFogRange;

		ProxyModel* mProxyModel;
	};
}
