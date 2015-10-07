#include "TransparencyMappingDemo.h"
#include "Game.h"
#include "GameException.h"
#include "ColorHelper.h"
#include "Camera.h"
#include "Utility.h"
#include "VectorHelper.h"
#include "DirectionalLight.h"
#include "ProxyModel.h"
#include "SOIL.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/common/transforms.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>

using namespace glm;

namespace Rendering
{
	//RTTI_DEFINITIONS(TransparencyMappingDemo)

	const vec2 TransparencyMappingDemo::LightRotationRate = vec2(360.0f, 360.0f);
	const float TransparencyMappingDemo::LightModulationRate = UCHAR_MAX;

	TransparencyMappingDemo::TransparencyMappingDemo(Game& game, Camera& camera)
		: DrawableGameComponent(game, camera), mShaderProgram(), mVertexArrayObject(0), mVertexBuffer(0),
		mIndexBuffer(0), mWorldMatrix(), mIndexCount(), mColorTexture(0), mAmbientLight(nullptr),
		mDirectionalLight(nullptr), mSpecularColor(ColorHelper::Black), mSpecularPower(25.0f),
		mFogColor(ColorHelper::CornflowerBlue), mFogStart(20.0f), mFogRange(40.0f), mAlphaMap(0),
		mTrilinearSampler(0), mProxyModel(nullptr), m_input_data(game.getInputData()), mContentFolder(game.getGameContentFolder())
	{
	}

	TransparencyMappingDemo::~TransparencyMappingDemo()
	{
		glDeleteSamplers(1, &mTrilinearSampler);
		glDeleteTextures(1, &mAlphaMap);
		DeleteObject(mProxyModel);
		DeleteObject(mDirectionalLight);
		DeleteObject(mAmbientLight);
		glDeleteTextures(1, &mColorTexture);
		glDeleteBuffers(1, &mIndexBuffer);
		glDeleteBuffers(1, &mVertexBuffer);
		glDeleteVertexArrays(1, &mVertexArrayObject);
	}

	void TransparencyMappingDemo::Initialize()
	{
		//SetCurrentDirectory(Utility::ExecutableDirectory().c_str());

		// Build the shader program
		std::vector<ShaderDefinition> shaders;
		shaders.push_back(ShaderDefinition(GL_VERTEX_SHADER, mContentFolder + L"\\TransparencyMappingDemo.vert"));
		shaders.push_back(ShaderDefinition(GL_FRAGMENT_SHADER, mContentFolder + L"\\TransparencyMappingDemo.frag"));
		mShaderProgram.BuildProgram(shaders);

		// Create the vertex buffer
		VertexPositionTextureNormal vertices[] =
		{
			VertexPositionTextureNormal(vec4(-1.0f, 0.1f, 0.0f, 1.0f), vec2(0.0f, 1.0f), Vector3Helper::Backward),
			VertexPositionTextureNormal(vec4(-1.0f, 2.1f, 0.0f, 1.0f), vec2(0.0f, 0.0f), Vector3Helper::Backward),
			VertexPositionTextureNormal(vec4(1.0f, 2.1f, 0.0f, 1.0f), vec2(1.0f, 0.0f), Vector3Helper::Backward),
			VertexPositionTextureNormal(vec4(1.0f, 0.1f, 0.0f, 1.0f), vec2(1.0f, 1.0f), Vector3Helper::Backward)
		};
        
        
//        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//        pcl::io::loadPCDFile(m_input_data, *cloud);
//        
//        for (size_t i = 0; i < cloud->points.size(); i++) {
//            float tmp_val = cloud->points[i].y;
//            cloud->points[i].y = cloud->points[i].z;
//            cloud->points[i].z = tmp_val;
//        }
//        
//        pcl::PointXYZRGBNormal min_pnt, max_pnt;
//        pcl::getMinMax3D(*cloud, min_pnt, max_pnt);
//        for (size_t i = 0; i < cloud->points.size(); i++)
//        {
//            cloud->points[i].y = cloud->points[i].y - min_pnt.y;
//        }
//        
//        
//        		// Create the vertex buffer using the extremes of the segmented cloud
//        		VertexPositionTextureNormal vertices[] =
//        		{
//        			VertexPositionTextureNormal(vec4(min_pnt.x, min_pnt.y, min_pnt.z, 1.0f), vec2(0.0f, 1.0f), Vector3Helper::Backward),
//        			VertexPositionTextureNormal(vec4(min_pnt.x, max_pnt.y, min_pnt.z, 1.0f), vec2(0.0f, 0.0f), Vector3Helper::Backward),
//        			VertexPositionTextureNormal(vec4(max_pnt.x, max_pnt.y, max_pnt.z, 1.0f), vec2(1.0f, 0.0f), Vector3Helper::Backward),
//        			VertexPositionTextureNormal(vec4(max_pnt.x, min_pnt.y, max_pnt.z, 1.0f), vec2(1.0f, 1.0f), Vector3Helper::Backward)
//        		};
//        
//        
        

		mShaderProgram.CreateVertexBuffer(vertices, sizeof(vertices)/sizeof(VertexPositionTextureNormal), mVertexBuffer);

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

		// Load the color texture
		mColorTexture = SOIL_load_OGL_texture("C:\\williamnguatem\\Projects\\Simple3DViewer\\game_content\\IMG_8748.JPG", SOIL_LOAD_AUTO, SOIL_CREATE_NEW_ID, SOIL_FLAG_MIPMAPS | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
		if (mColorTexture == 0)
		{
			throw GameException("SOIL_load_OGL_texture() failed.");
		}

//		// Load the alpha map
//		//mAlphaMap = SOIL_load_OGL_texture("AlphaMap_32bpp.png", SOIL_LOAD_AUTO, SOIL_CREATE_NEW_ID, SOIL_FLAG_MIPMAPS | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
//		mAlphaMap = SOIL_load_OGL_texture("leaf-alpha.png", SOIL_LOAD_AUTO, SOIL_CREATE_NEW_ID, SOIL_FLAG_MIPMAPS | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
//		if (mAlphaMap == 0)
//		{
//			throw GameException("SOIL_load_OGL_texture() failed.");
//		}

		// Create the trilinear texture sampler
		glGenSamplers(1, &mTrilinearSampler);
		glSamplerParameteri(mTrilinearSampler, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glSamplerParameteri(mTrilinearSampler, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glSamplerParameteri(mTrilinearSampler, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glSamplerParameteri(mTrilinearSampler, GL_TEXTURE_WRAP_T, GL_REPEAT);

		// Create the vertex array object
		glGenVertexArrays(1, &mVertexArrayObject);
		mShaderProgram.Initialize(mVertexArrayObject);
		glBindVertexArray(0);

		mAmbientLight = new Light(*mGame);
		mAmbientLight->SetColor(ColorHelper::Black);

		mDirectionalLight = new DirectionalLight(*mGame);

		mProxyModel = new ProxyModel(*mGame, *mCamera, "C:\\williamnguatem\\Projects\\Simple3DViewer\\game_content\\DirectionalLightProxy.obj", 0.5f);
		mProxyModel->Initialize();
		mProxyModel->SetPosition(10.0f, 0.0, 0.0f);
		mProxyModel->ApplyRotation(rotate(mat4(), 90.0f, Vector3Helper::Up));

		mWorldMatrix = scale(mat4(), vec3(5.0f));
	}

	void TransparencyMappingDemo::Update(const GameTime& gameTime)
	{
		UpdateAmbientLight(gameTime);
		UpdateDirectionalLight(gameTime);
		UpdateSpecularLight(gameTime);

		mProxyModel->Update(gameTime);
	}

	void TransparencyMappingDemo::Draw(const GameTime& gameTime)
	{
		glBindVertexArray(mVertexArrayObject);
		glBindBuffer(GL_ARRAY_BUFFER, mVertexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndexBuffer);

		mShaderProgram.Use();

		mat4 wvp = mCamera->ViewProjectionMatrix() * mWorldMatrix;
		mShaderProgram.WorldViewProjection() << wvp;
		mShaderProgram.World() << mWorldMatrix;
		mShaderProgram.AmbientColor() << mAmbientLight->Color();
		mShaderProgram.LightColor() << mDirectionalLight->Color();
		mShaderProgram.LightDirection() << mDirectionalLight->Direction();
		mShaderProgram.CameraPosition() << mCamera->Position();
		mShaderProgram.SpecularColor() << mSpecularColor;
		mShaderProgram.SpecularPower() << mSpecularPower;
		mShaderProgram.FogColor() << mFogColor;
		mShaderProgram.FogStart() << mFogStart;
		mShaderProgram.FogRange() << mFogRange;

		glBindSampler(0, mTrilinearSampler);
		glBindSampler(1, mTrilinearSampler);

		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, mColorTexture);

//		glActiveTexture(GL_TEXTURE1);
//		glBindTexture(GL_TEXTURE_2D, mAlphaMap);

		glEnable(GL_CULL_FACE);
		glFrontFace(GL_CCW);
        
        //glDisable(GL_LIGHTING);

		glEnable(GL_BLEND);
		//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glBlendFunc(GL_ONE_MINUS_DST_COLOR,GL_ONE_MINUS_SRC_ALPHA);

		glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
        
        
		glDisable(GL_BLEND);
        /* restore old values */
        //glEnable(GL_LIGHTING);

		glBindVertexArray(0);

		mProxyModel->Draw(gameTime);
	}

	void TransparencyMappingDemo::UpdateAmbientLight(const GameTime& gameTime)
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

	void TransparencyMappingDemo::UpdateDirectionalLight(const GameTime& gameTime)
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

	void TransparencyMappingDemo::UpdateSpecularLight(const GameTime& gameTime)
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
}