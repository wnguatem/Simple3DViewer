#pragma once

#include "DrawableGameComponent.h"
#include "TransparencyMappingEffect.h"

namespace Library
{
	class Light;
	class DirectionalLight;
	class ProxyModel;
}

using namespace Library;

namespace Rendering
{
	class TransparencyMappingDemo : public DrawableGameComponent
	{
		//RTTI_DECLARATIONS(TransparencyMappingDemo, DrawableGameComponent)

	public:
		TransparencyMappingDemo(Game& game, Camera& camera);
		~TransparencyMappingDemo();

		virtual void Initialize() override;
		virtual void Update(const GameTime& gameTime) override;
		virtual void Draw(const GameTime& gameTime) override;

	private:
		TransparencyMappingDemo();
		TransparencyMappingDemo(const TransparencyMappingDemo& rhs);
		TransparencyMappingDemo& operator=(const TransparencyMappingDemo& rhs);

		void UpdateAmbientLight(const GameTime& gameTime);
		void UpdateDirectionalLight(const GameTime& gameTime);
		void UpdateSpecularLight(const GameTime& gameTime);

		static const glm::vec2 LightRotationRate;
		static const float LightModulationRate;

		TransparencyMappingEffect mShaderProgram;
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
		GLuint mAlphaMap;
		GLuint mTrilinearSampler;
        
        std::string m_input_data;

		ProxyModel* mProxyModel;
	};
}
