#include "NormalMappingDemo.h"
#include "Game.h"
#include "GameException.h"
#include "ColorHelper.h"
#include "Camera.h"
#include "Utility.h"
#include "VectorHelper.h"
#include "DirectionalLight.h"
#include "ProxyModel.h"
#include "SOIL.h"

using namespace glm;

namespace Rendering
{
	//RTTI_DEFINITIONS(NormalMappingDemo)

	const vec2 NormalMappingDemo::LightRotationRate = vec2(360.0f, 360.0f);
	const float NormalMappingDemo::LightModulationRate = UCHAR_MAX;

	NormalMappingDemo::NormalMappingDemo(Game& game, Camera& camera)
		: DrawableGameComponent(game, camera), mNormalMappingEffect(), mFogEffect(), mNormalMappingVAO(0), mFogVAO(0),
		mNormalMappingVertexBuffer(0), mFogVertexBuffer(0), mIndexBuffer(0), mWorldMatrix(), mIndexCount(), mColorTexture(0), mAmbientLight(nullptr),
		mDirectionalLight(nullptr), mSpecularColor(ColorHelper::Black), mSpecularPower(25.0f),
		mFogColor(ColorHelper::CornflowerBlue), mFogStart(20.0f), mFogRange(40.0f),
		mNormalMap(0), mTrilinearSampler(0), mProxyModel(nullptr), mShowNormalMapping(true), mKeyboardHandler(nullptr)
	{
	}

	NormalMappingDemo::~NormalMappingDemo()
	{
		mGame->RemoveKeyboardHandler(mKeyboardHandler);
		glDeleteSamplers(1, &mTrilinearSampler);
		glDeleteTextures(1, &mNormalMap);
		DeleteObject(mProxyModel);
		DeleteObject(mDirectionalLight);
		DeleteObject(mAmbientLight);
		glDeleteTextures(1, &mColorTexture);
		glDeleteBuffers(1, &mIndexBuffer);
		glDeleteBuffers(1, &mNormalMappingVertexBuffer);
		glDeleteBuffers(1, &mFogVertexBuffer);
		glDeleteVertexArrays(1, &mFogVAO);
		glDeleteVertexArrays(1, &mNormalMappingVAO);
	}

	void NormalMappingDemo::Initialize()
	{
		//SetCurrentDirectory(Utility::ExecutableDirectory().c_str());

		// Build the shader programs
		std::vector<ShaderDefinition> shaders;
		shaders.push_back(ShaderDefinition(GL_VERTEX_SHADER, L"NormalMappingDemo.vert"));
		shaders.push_back(ShaderDefinition(GL_FRAGMENT_SHADER, L"NormalMappingDemo.frag"));
		mNormalMappingEffect.BuildProgram(shaders);

		shaders.clear();
		shaders.push_back(ShaderDefinition(GL_VERTEX_SHADER, L"FogDemo.vert"));
		shaders.push_back(ShaderDefinition(GL_FRAGMENT_SHADER, L"FogDemo.frag"));
		mFogEffect.BuildProgram(shaders);

		// Load the color texture
		mColorTexture = SOIL_load_OGL_texture("Blocks_COLOR.tga", SOIL_LOAD_AUTO, SOIL_CREATE_NEW_ID, SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
		if (mColorTexture == 0)
		{
			throw GameException("SOIL_load_OGL_texture() failed.");
		}

        
//        int img_width1, img_height1, channels;
//      unsigned char* img1 = SOIL_load_image
//        (
//         "Blocks_COLOR.tga",
//         &img_width1, &img_height1, &channels,
//         SOIL_LOAD_L
//         );
//        glGenTextures(1, &mColorTexture);
//        glBindTexture(GL_TEXTURE_2D, mColorTexture);
//        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img_width1, img_height1, 0, GL_RGBA, GL_UNSIGNED_BYTE, img1);
//        
        
		// Load the normal map
		mNormalMap = SOIL_load_OGL_texture("Blocks_NORM.tga", SOIL_LOAD_AUTO, SOIL_CREATE_NEW_ID, SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
		if (mNormalMap == 0)
		{
			throw GameException("SOIL_load_OGL_texture() failed.");
		}
//
//        unsigned char* img2 = SOIL_load_image("/Users/williamnguatem/projects/LODGenerator/Simple3DViewer/src/Blocks_NORM.tga", &img_width1, &img_height1, &channels, SOIL_LOAD_L);
//        glGenTextures(1, &mNormalMap);
//        glBindTexture(GL_TEXTURE_2D, mNormalMap);
//        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_width1, img_height1, 0, GL_RGB, GL_UNSIGNED_BYTE, img2);
        

		// Create the trilinear texture sampler
		glGenSamplers(1, &mTrilinearSampler);
		glSamplerParameteri(mTrilinearSampler, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glSamplerParameteri(mTrilinearSampler, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glSamplerParameteri(mTrilinearSampler, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glSamplerParameteri(mTrilinearSampler, GL_TEXTURE_WRAP_T, GL_REPEAT);

		// Create the normal mapping vertex array object
		glGenVertexArrays(1, &mNormalMappingVAO);
		glBindVertexArray(mNormalMappingVAO);

		// Create the normal mapping vertex buffer
		VertexPositionTextureNormalTangentBinormal normalMappingVertices[] =
		{
			VertexPositionTextureNormalTangentBinormal(vec4(-1.0f, 0.1f, 0.0f, 1.0f), vec2(0.0f, 1.0f), Vector3Helper::Backward, Vector3Helper::Down, Vector3Helper::Right),
			VertexPositionTextureNormalTangentBinormal(vec4(-1.0f, 2.1f, 0.0f, 1.0f), vec2(0.0f, 0.0f), Vector3Helper::Backward, Vector3Helper::Down, Vector3Helper::Right),
			VertexPositionTextureNormalTangentBinormal(vec4(1.0f, 2.1f, 0.0f, 1.0f), vec2(1.0f, 0.0f), Vector3Helper::Backward, Vector3Helper::Down, Vector3Helper::Right),
			VertexPositionTextureNormalTangentBinormal(vec4(1.0f, 0.1f, 0.0f, 1.0f), vec2(1.0f, 1.0f), Vector3Helper::Backward, Vector3Helper::Down, Vector3Helper::Right)
		};

		mNormalMappingEffect.CreateVertexBuffer(normalMappingVertices, sizeof(normalMappingVertices)/sizeof(VertexPositionTextureNormalTangentBinormal), mNormalMappingVertexBuffer);
		mNormalMappingEffect.Initialize(mNormalMappingVAO);

		// Create the fog vertex array object
		glGenVertexArrays(1, &mFogVAO);
		glBindVertexArray(mFogVAO);
		
		// Create the fog vertex buffer
		VertexPositionTextureNormal fogVertices[] =
		{
			VertexPositionTextureNormal(vec4(-1.0f, 0.1f, 0.0f, 1.0f), vec2(0.0f, 1.0f), Vector3Helper::Backward),
			VertexPositionTextureNormal(vec4(-1.0f, 2.1f, 0.0f, 1.0f), vec2(0.0f, 0.0f), Vector3Helper::Backward),
			VertexPositionTextureNormal(vec4(1.0f, 2.1f, 0.0f, 1.0f), vec2(1.0f, 0.0f), Vector3Helper::Backward),
			VertexPositionTextureNormal(vec4(1.0f, 0.1f, 0.0f, 1.0f), vec2(1.0f, 1.0f), Vector3Helper::Backward)
		};

		mFogEffect.CreateVertexBuffer(fogVertices, sizeof(fogVertices)/sizeof(VertexPositionTextureNormal), mFogVertexBuffer);
		mFogEffect.Initialize(mFogVAO);
		glBindVertexArray(0);
		
		// Create the index buffer
		unsigned int indices[] =
		{
			0, 2, 1,
			0, 3, 2
		};

		mIndexCount = sizeof(indices)/sizeof(unsigned int);

		glGenBuffers(1, &mIndexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndexBuffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)* mIndexCount, indices, GL_STATIC_DRAW);

		mAmbientLight = new Light(*mGame);
		mAmbientLight->SetColor(ColorHelper::Black);

		mDirectionalLight = new DirectionalLight(*mGame);

		mProxyModel = new ProxyModel(*mGame, *mCamera, "DirectionalLightProxy.obj", 0.5f);
		mProxyModel->Initialize();
		mProxyModel->SetPosition(10.0f, 0.0, 0.0f);
		mProxyModel->ApplyRotation(rotate(mat4(), 90.0f, Vector3Helper::Up));

		mWorldMatrix = scale(mat4(), vec3(5.0f));

		// Attach the keyboard handler
		using namespace std::placeholders;
		mKeyboardHandler = std::bind(&NormalMappingDemo::OnKey, this, _1, _2, _3, _4);
		mGame->AddKeyboardHandler(mKeyboardHandler);
	}

	void NormalMappingDemo::Update(const GameTime& gameTime)
	{
		UpdateAmbientLight(gameTime);
		UpdateDirectionalLight(gameTime);
		UpdateSpecularLight(gameTime);

		mProxyModel->Update(gameTime);
	}

	void NormalMappingDemo::Draw(const GameTime& gameTime)
	{
		if (mShowNormalMapping)
		{
			glBindVertexArray(mNormalMappingVAO);
			glBindBuffer(GL_ARRAY_BUFFER, mNormalMappingVertexBuffer);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndexBuffer);

			mNormalMappingEffect.Use();

			mat4 wvp = mCamera->ViewProjectionMatrix() * mWorldMatrix;
			mNormalMappingEffect.WorldViewProjection() << wvp;
			mNormalMappingEffect.World() << mWorldMatrix;
			mNormalMappingEffect.AmbientColor() << mAmbientLight->Color();
			mNormalMappingEffect.LightColor() << mDirectionalLight->Color();
			mNormalMappingEffect.LightDirection() << mDirectionalLight->Direction();
			mNormalMappingEffect.CameraPosition() << mCamera->Position();
			mNormalMappingEffect.SpecularColor() << mSpecularColor;
			mNormalMappingEffect.SpecularPower() << mSpecularPower;
			mNormalMappingEffect.FogColor() << mFogColor;
			mNormalMappingEffect.FogStart() << mFogStart;
			mNormalMappingEffect.FogRange() << mFogRange;

			glBindSampler(0, mTrilinearSampler);
			glBindSampler(1, mTrilinearSampler);

			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, mColorTexture);

			glActiveTexture(GL_TEXTURE1);
			glBindTexture(GL_TEXTURE_2D, mNormalMap);
		}
		else
		{
			glBindVertexArray(mFogVAO);
			glBindBuffer(GL_ARRAY_BUFFER, mFogVertexBuffer);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndexBuffer);

			mFogEffect.Use();

			mat4 wvp = mCamera->ViewProjectionMatrix() * mWorldMatrix;
			mFogEffect.WorldViewProjection() << wvp;
			mFogEffect.World() << mWorldMatrix;
			mFogEffect.AmbientColor() << mAmbientLight->Color();
			mFogEffect.LightColor() << mDirectionalLight->Color();
			mFogEffect.LightDirection() << mDirectionalLight->Direction();
			mFogEffect.CameraPosition() << mCamera->Position();
			mFogEffect.SpecularColor() << mSpecularColor;
			mFogEffect.SpecularPower() << mSpecularPower;
			mFogEffect.FogColor() << mFogColor;
			mFogEffect.FogStart() << mFogStart;
			mFogEffect.FogRange() << mFogRange;

			glBindSampler(0, mTrilinearSampler);
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, mColorTexture);
		}

		glEnable(GL_CULL_FACE);
		glFrontFace(GL_CCW);

		glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
		glBindVertexArray(0);

		mProxyModel->Draw(gameTime);
	}

	void NormalMappingDemo::UpdateAmbientLight(const GameTime& gameTime)
	{
		static float ambientIntensity = 0.0f;

		if (glfwGetKey(mGame->Window(), GLFW_KEY_PAGE_UP) && ambientIntensity < 1.0f)
		{
			ambientIntensity += (float)gameTime.ElapsedGameTime();
			ambientIntensity = min(ambientIntensity, 1.0f);

			mAmbientLight->SetColor(vec4((vec3)ambientIntensity, 1.0f));
		}

		if (glfwGetKey(mGame->Window(), GLFW_KEY_PAGE_DOWN) && ambientIntensity > 0.0f)
		{
			ambientIntensity -= (float)gameTime.ElapsedGameTime();
			ambientIntensity = max(ambientIntensity, 0.0f);

			mAmbientLight->SetColor(vec4((vec3)ambientIntensity, 1.0f));
		}
	}

	void NormalMappingDemo::UpdateDirectionalLight(const GameTime& gameTime)
	{
		static float directionalIntensity = 1.0f;
		float elapsedTime = (float)gameTime.ElapsedGameTime();

		// Upddate directional light intensity		
		if (glfwGetKey(mGame->Window(), GLFW_KEY_HOME) && directionalIntensity < 1.0f)
		{
			directionalIntensity += elapsedTime;
			directionalIntensity = min(directionalIntensity, 1.0f);

			mDirectionalLight->SetColor(vec4((vec3)directionalIntensity, 1.0f));
		}
		if (glfwGetKey(mGame->Window(), GLFW_KEY_END) && directionalIntensity > 0.0f)
		{
			directionalIntensity -= elapsedTime;
			directionalIntensity = max(directionalIntensity, 0.0f);

			mDirectionalLight->SetColor(vec4((vec3)directionalIntensity, 1.0f));
		}

		// Rotate directional light
		vec2 rotationAmount = Vector2Helper::Zero;
		if (glfwGetKey(mGame->Window(), GLFW_KEY_LEFT))
		{
			rotationAmount.x += LightRotationRate.x * elapsedTime;
		}
		if (glfwGetKey(mGame->Window(), GLFW_KEY_RIGHT))
		{
			rotationAmount.x -= LightRotationRate.x * elapsedTime;
		}
		if (glfwGetKey(mGame->Window(), GLFW_KEY_UP))
		{
			rotationAmount.y += LightRotationRate.y * elapsedTime;
		}
		if (glfwGetKey(mGame->Window(), GLFW_KEY_DOWN))
		{
			rotationAmount.y -= LightRotationRate.y * elapsedTime;
		}

		mat4 lightRotationMatrix;
		if (rotationAmount.x != 0)
		{
			lightRotationMatrix = rotate(mat4(), rotationAmount.x, Vector3Helper::Up);
		}

		if (rotationAmount.y != 0)
		{
			lightRotationMatrix = rotate(lightRotationMatrix, rotationAmount.y, mDirectionalLight->Right());
		}

		if (rotationAmount.x != 0.0f || rotationAmount.y != 0.0f)
		{
			mDirectionalLight->ApplyRotation(lightRotationMatrix);
			mProxyModel->ApplyRotation(lightRotationMatrix);
		}
	}

	void NormalMappingDemo::UpdateSpecularLight(const GameTime& gameTime)
	{
		static float specularIntensity = 0.0f;

		if (glfwGetKey(mGame->Window(), GLFW_KEY_INSERT) && specularIntensity < 1.0f)
		{
			specularIntensity += (float)gameTime.ElapsedGameTime();
			specularIntensity = min(specularIntensity, 1.0f);

			mSpecularColor = (vec4((vec3)specularIntensity, 1.0f));
		}

		if (glfwGetKey(mGame->Window(), GLFW_KEY_DELETE) && specularIntensity > 0.0f)
		{
			specularIntensity -= (float)gameTime.ElapsedGameTime();
			specularIntensity = max(specularIntensity, 0.0f);

			mSpecularColor = (vec4((vec3)specularIntensity, 1.0f));
		}

		static float specularPower = mSpecularPower;

		if (glfwGetKey(mGame->Window(), GLFW_KEY_O) && specularPower < UCHAR_MAX)
		{
			specularPower += LightModulationRate * (float)gameTime.ElapsedGameTime();
			specularPower = min(specularPower, static_cast<float>(UCHAR_MAX));

			mSpecularPower = specularPower;
		}

		if (glfwGetKey(mGame->Window(), GLFW_KEY_P) && specularPower > 0.0f)
		{
			specularPower -= LightModulationRate * (float)gameTime.ElapsedGameTime();
			specularPower = max(specularPower, 0.0f);

			mSpecularPower = specularPower;
		}
	}

	void NormalMappingDemo::OnKey(int key, int scancode, int action, int mods)
	{
		if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
		{
			mShowNormalMapping = !mShowNormalMapping;
		}
	}
}