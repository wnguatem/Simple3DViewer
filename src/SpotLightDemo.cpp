#include "SpotLightDemo.h"
#include "Game.h"
#include "GameException.h"
#include "ColorHelper.h"
#include "Camera.h"
#include "Utility.h"
#include "VertexDeclarations.h"
#include "VectorHelper.h"
#include "SpotLight.h"
#include "ProxyModel.h"
#include "SOIL.h"

using namespace glm;

namespace Rendering
{
	//RTTI_DEFINITIONS(SpotLightDemo)

		const float SpotLightDemo::LightModulationRate = UCHAR_MAX;
	const float SpotLightDemo::LightMovementRate = 10.0f;
	const vec2 SpotLightDemo::LightRotationRate = vec2(360.0f, 360.0f);

	SpotLightDemo::SpotLightDemo(Game& game, Camera& camera)
		: DrawableGameComponent(game, camera), mShaderProgram(), mVertexArrayObject(0), mVertexBuffer(0),
		mIndexBuffer(0), mWorldMatrix(), mIndexCount(), mColorTexture(0), mAmbientLight(nullptr),
		mSpotLight(nullptr), mSpecularColor(ColorHelper::White), mSpecularPower(25.0f),
		mProxyModel(nullptr)
	{
	}

	SpotLightDemo::~SpotLightDemo()
	{
		DeleteObject(mProxyModel);
		DeleteObject(mSpotLight);
		DeleteObject(mAmbientLight);
		glDeleteTextures(1, &mColorTexture);
		glDeleteBuffers(1, &mIndexBuffer);
		glDeleteBuffers(1, &mVertexBuffer);
		glDeleteVertexArrays(1, &mVertexArrayObject);
	}

	void SpotLightDemo::Initialize()
	{
		//SetCurrentDirectory(Utility::ExecutableDirectory().c_str());

		// Build the shader program
		std::vector<ShaderDefinition> shaders;
		shaders.push_back(ShaderDefinition(GL_VERTEX_SHADER, L"SpotLightDemo.vert"));
		shaders.push_back(ShaderDefinition(GL_FRAGMENT_SHADER, L"SpotLightDemo.frag"));
		mShaderProgram.BuildProgram(shaders);

		// Create the vertex buffer
		VertexPositionTextureNormal vertices[] =
		{
			VertexPositionTextureNormal(vec4(-1.0f, 0.1f, 0.0f, 1.0f), vec2(0.0f, 1.0f), Vector3Helper::Backward),
			VertexPositionTextureNormal(vec4(-1.0f, 2.1f, 0.0f, 1.0f), vec2(0.0f, 0.0f), Vector3Helper::Backward),
			VertexPositionTextureNormal(vec4(1.0f, 2.1f, 0.0f, 1.0f), vec2(1.0f, 0.0f), Vector3Helper::Backward),
			VertexPositionTextureNormal(vec4(1.0f, 0.1f, 0.0f, 1.0f), vec2(1.0f, 1.0f), Vector3Helper::Backward)
		};

		mShaderProgram.CreateVertexBuffer(vertices, sizeof(vertices)/sizeof(VertexPositionTextureNormal), mVertexBuffer);

		unsigned int indices[] =
		{
			0, 2, 1,
			0, 3, 2
		};

        mIndexCount = sizeof(indices)/sizeof(unsigned int);//(indices);

		glGenBuffers(1, &mIndexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndexBuffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)* mIndexCount, indices, GL_STATIC_DRAW);

		// Load the texture
		mColorTexture = SOIL_load_OGL_texture("Checkerboard.png", SOIL_LOAD_AUTO, SOIL_CREATE_NEW_ID, SOIL_FLAG_MIPMAPS | SOIL_FLAG_NTSC_SAFE_RGB |SOIL_FLAG_COMPRESS_TO_DXT);
        		mColorTexture = SOIL_load_OGL_texture("Checkerboard.png", SOIL_LOAD_AUTO, SOIL_CREATE_NEW_ID, SOIL_FLAG_NTSC_SAFE_RGB);
		if (mColorTexture == 0)
		{
			throw GameException("SOIL_load_OGL_texture() failed.");
		}
        
//        int img_width1, img_height1;
//        unsigned char* img1 = SOIL_load_image("Checkerboard.png", &img_width1, &img_height1, NULL, 0);
//        glGenTextures(1, &mColorTexture);
//        glBindTexture(GL_TEXTURE_2D, mColorTexture);
//        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_width1, img_height1, 0, GL_RGB, GL_UNSIGNED_BYTE, img1);
        
        

		// Create the vertex array object
		glGenVertexArrays(1, &mVertexArrayObject);
		mShaderProgram.Initialize(mVertexArrayObject);
		glBindVertexArray(0);

		mAmbientLight = new Light(*mGame);
		mAmbientLight->SetColor(ColorHelper::Black);

		mSpotLight = new SpotLight(*mGame);
		mSpotLight->SetRadius(500.0f);
		mSpotLight->SetPosition(5.0f, 0.0f, 10.0f);

		mProxyModel = new ProxyModel(*mGame, *mCamera, "SpotLightProxy.obj", 0.5f);
		mProxyModel->Initialize();
		mProxyModel->ApplyRotation(rotate(mat4(), 90.0f, Vector3Helper::Right));

		mWorldMatrix = scale(mat4(), vec3(5.0f));
	}

	void SpotLightDemo::Update(const GameTime& gameTime)
	{
		UpdateAmbientLight(gameTime);
		UpdateSpotLight(gameTime);
		UpdateSpecularLight(gameTime);

		mProxyModel->Update(gameTime);
	}

	void SpotLightDemo::Draw(const GameTime& gameTime)
	{
		glBindVertexArray(mVertexArrayObject);
		glBindBuffer(GL_ARRAY_BUFFER, mVertexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndexBuffer);

		mShaderProgram.Use();

		mat4 wvp = mCamera->ViewProjectionMatrix() * mWorldMatrix;
		mShaderProgram.WorldViewProjection() << wvp;
		mShaderProgram.World() << mWorldMatrix;
		mShaderProgram.AmbientColor() << mAmbientLight->Color();
		mShaderProgram.LightColor() << mSpotLight->Color();
		mShaderProgram.LightPosition() << mSpotLight->Position();
		mShaderProgram.LightRadius() << mSpotLight->Radius();
		mShaderProgram.LightLookAt() << mSpotLight->Direction();
		mShaderProgram.CameraPosition() << mCamera->Position();
		mShaderProgram.SpecularColor() << mSpecularColor;
		mShaderProgram.SpecularPower() << mSpecularPower;
		mShaderProgram.SpotLightInnerAngle() << mSpotLight->InnerAngle();
		mShaderProgram.SpotLightOuterAngle() << mSpotLight->OuterAngle();

		glBindTexture(GL_TEXTURE_2D, mColorTexture);

		glEnable(GL_CULL_FACE);
		glFrontFace(GL_CCW);

		glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
		glBindVertexArray(0);

		mProxyModel->Draw(gameTime);
	}

	void SpotLightDemo::UpdateAmbientLight(const GameTime& gameTime)
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

	void SpotLightDemo::UpdateSpotLight(const GameTime& gameTime)
	{
		static float directionalIntensity = 1.0f;
		float elapsedTime = (float)gameTime.ElapsedGameTime();

		// Upddate directional light intensity		
		if (glfwGetKey(mGame->Window(), GLFW_KEY_HOME) && directionalIntensity < 1.0f)
		{
			directionalIntensity += elapsedTime;
			directionalIntensity = min(directionalIntensity, 1.0f);

			mSpotLight->SetColor(vec4((vec3)directionalIntensity, 1.0f));
		}
		if (glfwGetKey(mGame->Window(), GLFW_KEY_END) && directionalIntensity > 0.0f)
		{
			directionalIntensity -= elapsedTime;
			directionalIntensity = max(directionalIntensity, 0.0f);

			mSpotLight->SetColor(vec4((vec3)directionalIntensity, 1.0f));
		}

		// Move point light
		vec3 movementAmount = Vector3Helper::Zero;
		if (glfwGetKey(mGame->Window(), GLFW_KEY_J))
		{
			movementAmount.x = -1.0f;
		}

		if (glfwGetKey(mGame->Window(), GLFW_KEY_L))
		{
			movementAmount.x = 1.0f;
		}

		if (glfwGetKey(mGame->Window(), GLFW_KEY_I))
		{
			movementAmount.y = 1.0f;
		}

		if (glfwGetKey(mGame->Window(), GLFW_KEY_K))
		{
			movementAmount.y = -1.0f;
		}

		if (glfwGetKey(mGame->Window(), GLFW_KEY_N))
		{
			movementAmount.z = -1.0f;
		}

		if (glfwGetKey(mGame->Window(), GLFW_KEY_M))
		{
			movementAmount.z = 1.0f;
		}

		vec3 movement = movementAmount * LightMovementRate * elapsedTime;
		mSpotLight->SetPosition(mSpotLight->Position() + movement);
		mProxyModel->SetPosition(mSpotLight->Position());

		// Update the light's radius
		if (glfwGetKey(mGame->Window(), GLFW_KEY_V))
		{
			float radius = mSpotLight->Radius() + LightModulationRate * elapsedTime;
			mSpotLight->SetRadius(radius);
		}

		if (glfwGetKey(mGame->Window(), GLFW_KEY_B))
		{
			float radius = mSpotLight->Radius() - LightModulationRate * elapsedTime;
			radius = max(radius, 0.0f);
			mSpotLight->SetRadius(radius);
		}

		// Rotate spot light
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
			lightRotationMatrix = rotate(lightRotationMatrix, rotationAmount.y, mSpotLight->Right());
		}

		if (rotationAmount.x != 0.0f || rotationAmount.y != 0.0f)
		{
			mSpotLight->ApplyRotation(lightRotationMatrix);
			mProxyModel->ApplyRotation(lightRotationMatrix);
		}

		// Update inner and outer angles
		static float innerAngle = mSpotLight->InnerAngle();
		if (glfwGetKey(mGame->Window(), GLFW_KEY_Z) && innerAngle < 1.0f)
		{
			innerAngle += elapsedTime;
			innerAngle = min(innerAngle, 1.0f);

			mSpotLight->SetInnerAngle(innerAngle);
		}
		if (glfwGetKey(mGame->Window(), GLFW_KEY_X) && innerAngle > 0.5f)
		{
			innerAngle -= elapsedTime;
			innerAngle = max(innerAngle, 0.5f);

			mSpotLight->SetInnerAngle(innerAngle);
		}

		static float outerAngle = mSpotLight->OuterAngle();
		if (glfwGetKey(mGame->Window(), GLFW_KEY_C) && outerAngle < 0.5f)
		{
			outerAngle += elapsedTime;
			outerAngle = min(outerAngle, 0.5f);

			mSpotLight->SetOuterAngle(outerAngle);
		}
		if (glfwGetKey(mGame->Window(), GLFW_KEY_V) && outerAngle > 0.0f)
		{
			outerAngle -= elapsedTime;
			outerAngle = max(outerAngle, 0.0f);

			mSpotLight->SetOuterAngle(outerAngle);
		}
	}

	void SpotLightDemo::UpdateSpecularLight(const GameTime& gameTime)
	{
		static float specularIntensity = 1.0f;

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
}