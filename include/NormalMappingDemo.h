#pragma once

#include "DrawableGameComponent.h"
#include "NormalMappingEffect.h"
#include "FogEffect.h"
#include "Game.h"

namespace Library
{
	class Light;
	class DirectionalLight;
	class ProxyModel;
}

using namespace Library;

namespace Rendering
{
	class NormalMappingDemo : public DrawableGameComponent
	{
		//RTTI_DECLARATIONS(NormalMappingDemo, DrawableGameComponent)

	public:		
		NormalMappingDemo(Game& game, Camera& camera);
		~NormalMappingDemo();

		virtual void Initialize() override;
		virtual void Update(const GameTime& gameTime) override;
		virtual void Draw(const GameTime& gameTime) override;

	private:
		NormalMappingDemo();
		NormalMappingDemo(const NormalMappingDemo& rhs);
		NormalMappingDemo& operator=(const NormalMappingDemo& rhs);

		void UpdateAmbientLight(const GameTime& gameTime);
		void UpdateDirectionalLight(const GameTime& gameTime);
		void UpdateSpecularLight(const GameTime& gameTime);
		void OnKey(int key, int scancode, int action, int mods);

		static const glm::vec2 LightRotationRate;
		static const float LightModulationRate;

		NormalMappingEffect mNormalMappingEffect;
		FogEffect mFogEffect;
		GLuint mNormalMappingVAO;
		GLuint mFogVAO;
		GLuint mNormalMappingVertexBuffer;
		GLuint mFogVertexBuffer;
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
		GLuint mNormalMap;
		GLuint mTrilinearSampler;

		ProxyModel* mProxyModel;
		bool mShowNormalMapping;
		Game::KeyboardHandler mKeyboardHandler;
	};
}
