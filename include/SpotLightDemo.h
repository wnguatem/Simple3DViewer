#pragma once

#include "DrawableGameComponent.h"
#include "SpotLightEffect.h"

namespace Library
{
	class Light;
	class SpotLight;
	class ProxyModel;
}

using namespace Library;

namespace Rendering
{
	class SpotLightDemo : public DrawableGameComponent
	{
		//RTTI_DECLARATIONS(SpotLightDemo, DrawableGameComponent)

	public:		
		SpotLightDemo(Game& game, Camera& camera);
		~SpotLightDemo();

		virtual void Initialize() override;
		virtual void Update(const GameTime& gameTime) override;
		virtual void Draw(const GameTime& gameTime) override;

	private:
		SpotLightDemo();
		SpotLightDemo(const SpotLightDemo& rhs);
		SpotLightDemo& operator=(const SpotLightDemo& rhs);

		void UpdateAmbientLight(const GameTime& gameTime);
		void UpdateSpotLight(const GameTime& gameTime);
		void UpdateSpecularLight(const GameTime& gameTime);

		static const float LightModulationRate;
		static const float LightMovementRate;
		static const glm::vec2 LightRotationRate;

		SpotLightEffect mShaderProgram;
		GLuint mVertexArrayObject;
		GLuint mVertexBuffer;
		GLuint mIndexBuffer;
		glm::mat4 mWorldMatrix;
		GLuint mIndexCount;
		GLuint mColorTexture;
		Light* mAmbientLight;
		SpotLight* mSpotLight;
		glm::vec4 mSpecularColor;
		float mSpecularPower;

		ProxyModel* mProxyModel;
	};
}
