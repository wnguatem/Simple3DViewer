#include "Skybox.h"
#include "Game.h"
#include "GameException.h"
#include "Camera.h"
#include "VectorHelper.h"
#include "MatrixHelper.h"
#include "ColorHelper.h"
#include "Model.h"
#include "Mesh.h"
#include "Utility.h"
#include "VertexDeclarations.h"
#include "SOIL.h"

using namespace glm;

namespace Library
{
	////RTTI_DEFINITIONS(Skybox)

	Skybox::Skybox(Game& game, Camera& camera, const std::string& posXFilename, const std::string& negXFilename, const std::string& posYFilename, const std::string& negYFilename, const std::string& posZFilename, const std::string& negZFilename, float scale)
		: DrawableGameComponent(game, camera),
		mPosXFilename(posXFilename), mNegXFilename(negXFilename), mPosYFilename(posYFilename), mNegYFilename(negYFilename), mPosZFilename(posZFilename), mNegZFilename(negZFilename),
		mShaderProgram(), mVertexArrayObject(0), mVertexBuffer(0),
		mIndexBuffer(0), mIndexCount(0), mSkyboxTexture(0), mWorldMatrix(), mScaleMatrix()
	{
		mScaleMatrix = glm::scale(mat4(), vec3(scale));
	}

	Skybox::~Skybox()
	{
		glDeleteSamplers(1, &mSkyboxTextureSampler);
		glDeleteTextures(1,&mSkyboxTexture);
		glDeleteBuffers(1, &mIndexBuffer);
		glDeleteBuffers(1, &mVertexBuffer);
		glDeleteVertexArrays(1, &mVertexArrayObject);
	}

	void Skybox::Initialize()
	{
		//SetCurrentDirectory(Utility::ExecutableDirectory().c_str());

		// Build the shader program
		std::vector<ShaderDefinition> shaders;
		shaders.push_back(ShaderDefinition(GL_VERTEX_SHADER, L"gamecontent\\Skybox.vert"));
		shaders.push_back(ShaderDefinition(GL_FRAGMENT_SHADER, L"gamecontent\\Skybox.frag"));
		mShaderProgram.BuildProgram(shaders);

		std::unique_ptr<Model> model(new Model(*mGame, "Sphere.obj"));

		// Create the vertex and index buffers
		Mesh* mesh = model->Meshes().at(0);
		mShaderProgram.CreateVertexBuffer(*mesh, mVertexBuffer);
		mesh->CreateIndexBuffer(mIndexBuffer);
		mIndexCount = mesh->Indices().size();
	
		mSkyboxTexture = SOIL_load_OGL_cubemap(mPosXFilename.c_str(), mNegXFilename.c_str(), mPosYFilename.c_str(), mNegYFilename.c_str(), mPosZFilename.c_str(), mNegZFilename.c_str(), SOIL_LOAD_AUTO, SOIL_CREATE_NEW_ID, SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
		if (mSkyboxTexture == 0)
		{
			throw GameException("SOIL_load_OGL_cubemap() failed.");
		}

		glGenSamplers(1, &mSkyboxTextureSampler);
		glSamplerParameteri(mSkyboxTextureSampler, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glSamplerParameteri(mSkyboxTextureSampler, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glSamplerParameteri(mSkyboxTextureSampler, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glSamplerParameteri(mSkyboxTextureSampler, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glSamplerParameteri(mSkyboxTextureSampler, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
		
		// Create the vertex array object
		glGenVertexArrays(1, &mVertexArrayObject);
		mShaderProgram.Initialize(mVertexArrayObject);
	}

	void Skybox::Update(const GameTime& gameTime)
	{
		mWorldMatrix = translate(mat4(), mCamera->Position()) * mScaleMatrix;
	}

	void Skybox::Draw(const GameTime& gameTime)
	{
		glBindVertexArray(mVertexArrayObject);
		glBindBuffer(GL_ARRAY_BUFFER, mVertexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndexBuffer);
		glBindSampler(0, mSkyboxTextureSampler);

		mShaderProgram.Use();

		mat4 wvp = mCamera->ViewProjectionMatrix() * mWorldMatrix;
		mShaderProgram.WorldViewProjection() << wvp;
		
		glBindTexture(GL_TEXTURE_CUBE_MAP, mSkyboxTexture);

		glDisable(GL_CULL_FACE);

		glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
	}
}