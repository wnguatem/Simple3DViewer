//#include "TexturedModelDemo.h"
//#include "GameException.h"
//#include "ColorHelper.h"
//#include "Camera.h"
//#include "Utility.h"
//#include "ShaderProgram.h"
//#include "VertexDeclarations.h"
//#include "VectorHelper.h"
//#include "Model.h"
//#include "Mesh.h"
//#include "SOIL.h"
//
//using namespace glm;
//
//namespace Rendering
//{
//	////RTTI_DEFINITIONS(TexturedModelDemo)
//
//	TexturedModelDemo::TexturedModelDemo(Game& game, Camera& camera)
//		: DrawableGameComponent(game, camera), mKeyboardHandler(nullptr), mShaderProgram(), mVertexArrayObject(0), mVertexBuffer(0),
//		mIndexBuffer(0), mWorldViewProjectionLocation(-1), mWorldMatrix(), mIndexCount(), mColorTexture(0),
//		mAltTexture(0), mActiveTexture(0)
//	{
//	}
//
//	TexturedModelDemo::~TexturedModelDemo()
//	{
//		glDeleteTextures(1, &mAltTexture);
//		glDeleteTextures(1, &mColorTexture);
//		glDeleteBuffers(1, &mIndexBuffer);
//		glDeleteBuffers(1, &mVertexBuffer);
//		glDeleteVertexArrays(1, &mVertexArrayObject);
//
//		mGame->RemoveKeyboardHandler(mKeyboardHandler);
//	}
//
//
//	void TexturedModelDemo::Initialize()
//	{
//		//SetCurrentDirectory(Utility::ExecutableDirectory().c_str());
//
//		// Build the shader program
//		std::vector<ShaderDefinition> shaders;
//		shaders.push_back(ShaderDefinition(GL_VERTEX_SHADER, L"TexturedModelDemo.vert"));
//		shaders.push_back(ShaderDefinition(GL_FRAGMENT_SHADER, L"TexturedModelDemo.frag"));
//		mShaderProgram.BuildProgram(shaders);
//		
//		// Load the model
//		std::unique_ptr<Model> model(new Model(*mGame, "Sphere.obj", true));
//
//		// Create the vertex and index buffers
//		Mesh* mesh = model->Meshes().at(0);
//		CreateVertexBuffer(*mesh, mVertexBuffer);
//		mesh->CreateIndexBuffer(mIndexBuffer);
//		mIndexCount = mesh->Indices().size();
//		
//		mWorldViewProjectionLocation = glGetUniformLocation(mShaderProgram.Program(), "WorldViewProjection");
//		if (mWorldViewProjectionLocation == -1)
//		{
//			throw GameException("glGetUniformLocation() did not find uniform location.");
//		}
//
//		// Load the texture
//		mColorTexture = SOIL_load_OGL_texture("EarthComposite.jpg", SOIL_LOAD_AUTO, SOIL_CREATE_NEW_ID, SOIL_FLAG_MIPMAPS | SOIL_FLAG_NTSC_SAFE_RGB );
//		if (mColorTexture == 0)
//		{
//			throw GameException("SOIL_load_OGL_texture() failed.");
//		}
//
//		mAltTexture = SOIL_load_OGL_texture("Checkerboard.png", SOIL_LOAD_AUTO, SOIL_CREATE_NEW_ID, SOIL_FLAG_MIPMAPS | SOIL_FLAG_NTSC_SAFE_RGB);
//		if (mAltTexture == 0)
//		{
//			throw GameException("SOIL_load_OGL_texture() failed.");
//		}
//
//		mActiveTexture = mColorTexture;
//
//		// Create vertex array objects (VAOs)
//		glGenVertexArrays(1, &mVertexArrayObject);
//		glBindVertexArray(mVertexArrayObject);
//
//		glVertexAttribPointer(VertexAttributePosition, 4, GL_FLOAT, GL_FALSE, sizeof(VertexPositionTexture), (void*)offsetof(VertexPositionTexture, Position));
//		glEnableVertexAttribArray(VertexAttributePosition);
//
//		glVertexAttribPointer(VertexAttributeTextureCoordinate, 2, GL_FLOAT, GL_FALSE, sizeof(VertexPositionTexture), (void*)offsetof(VertexPositionTexture, TextureCoordinates));
//		glEnableVertexAttribArray(VertexAttributeTextureCoordinate);
//
//		glBindVertexArray(0);
//
//		using namespace std::placeholders;
//		mKeyboardHandler = std::bind(&TexturedModelDemo::OnKey, this, _1, _2, _3, _4);
//		mGame->AddKeyboardHandler(mKeyboardHandler);
//	}
//
//	void TexturedModelDemo::Draw(const GameTime& gameTime)
//	{
//		glBindVertexArray(mVertexArrayObject);
//		glBindBuffer(GL_ARRAY_BUFFER, mVertexBuffer);
//		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndexBuffer);		
//		glBindTexture(GL_TEXTURE_2D, mActiveTexture);
//
//		glUseProgram(mShaderProgram.Program());
//
//		mat4 wvp = mCamera->ViewProjectionMatrix() * mWorldMatrix;
//		glUniformMatrix4fv(mWorldViewProjectionLocation, 1, GL_FALSE, &wvp[0][0]);
//
//		glEnable(GL_CULL_FACE);
//		glFrontFace(GL_CCW);
//
//		glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
//		glBindVertexArray(0);
//	}
//
//	void TexturedModelDemo::CreateVertexBuffer(const Mesh& mesh, GLuint& vertexBuffer)
//	{
//		const std::vector<vec3>& sourceVertices = mesh.Vertices();
//
//		std::vector<VertexPositionTexture> vertices;
//		vertices.reserve(sourceVertices.size());
//
//		std::vector<vec3>* textureCoordinates = mesh.TextureCoordinates().at(0);
//		assert(textureCoordinates->size() == sourceVertices.size());
//
//		for (size_t i = 0; i < sourceVertices.size(); i++)
//		{
//			vec3 position = sourceVertices.at(i);
//			vec2 uv = (vec2)textureCoordinates->at(i);
//			vertices.push_back(VertexPositionTexture(vec4(position.x, position.y, position.z, 1.0f), uv));
//		}
//
//		glGenBuffers(1, &vertexBuffer);
//		glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
//		glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPositionTexture) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
//	}
//
//	void TexturedModelDemo::OnKey(int key, int scancode, int action, int mods)
//	{
//		if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
//		{
//			if (mActiveTexture == mColorTexture)
//			{
//				mActiveTexture = mAltTexture;
//			}
//			else
//			{
//				mActiveTexture = mColorTexture;
//			}
//		}
//	}
//}

#include "TexturedModelDemo.h"
#include "Game.h"
#include "GameException.h"
#include "ColorHelper.h"
#include "Camera.h"
#include "Utility.h"
#include "ShaderProgram.h"
#include "VertexDeclarations.h"
#include "VectorHelper.h"

using namespace glm;

namespace Rendering
{
	//RTTI_DEFINITIONS(CubeDemo)

		const GLfloat CubeDemo::RotationRate = 180.0f;

	CubeDemo::CubeDemo(Game& game, Camera& camera)
		: DrawableGameComponent(game, camera), mShaderProgram(), mVertexArrayObject(0), mVertexBuffer(0),
		mIndexBuffer(0), mWorldViewProjectionLocation(-1), mWorldMatrix()
	{
	}

	CubeDemo::~CubeDemo()
	{
		glDeleteBuffers(1, &mIndexBuffer);
		glDeleteBuffers(1, &mVertexBuffer);
		glDeleteVertexArrays(1, &mVertexArrayObject);
	}

	void CubeDemo::Initialize()
	{
		//SetCurrentDirectory(Utility::ExecutableDirectory().c_str());

		// Build the shader program
		std::vector<ShaderDefinition> shaders;
		shaders.push_back(ShaderDefinition(GL_VERTEX_SHADER, L"CubeDemo.vert"));
		shaders.push_back(ShaderDefinition(GL_FRAGMENT_SHADER, L"CubeDemo.frag"));
		mShaderProgram.BuildProgram(shaders);

		// Create the vertex buffer object
		VertexPositionColor vertices[] =
		{
			VertexPositionColor(vec4(-1.0f, +1.0f, -1.0f, 1.0f), ColorHelper::Green),
			VertexPositionColor(vec4(+1.0f, +1.0f, -1.0f, 1.0f), ColorHelper::Yellow),
			VertexPositionColor(vec4(+1.0f, +1.0f, +1.0f, 1.0f), ColorHelper::White),
			VertexPositionColor(vec4(-1.0f, +1.0f, +1.0f, 1.0f), ColorHelper::BlueGreen),

			VertexPositionColor(vec4(-1.0f, -1.0f, +1.0f, 1.0f), ColorHelper::Blue),
			VertexPositionColor(vec4(+1.0f, -1.0f, +1.0f, 1.0f), ColorHelper::Purple),
			VertexPositionColor(vec4(+1.0f, -1.0f, -1.0f, 1.0f), ColorHelper::Red),
			VertexPositionColor(vec4(-1.0f, -1.0f, -1.0f, 1.0f), ColorHelper::Black)
		};

		glGenBuffers(1, &mVertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, mVertexBuffer);
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

		GLuint indices[] =
		{
			0, 2, 1,
			0, 3, 2,

			4, 6, 5,
			4, 7, 6,

			3, 5, 2,
			3, 4, 5,

			2, 6, 1,
			2, 5, 6,

			1, 6, 7,
			1, 7, 0,

			0, 4, 3,
			0, 7, 4
		};

		glGenBuffers(1, &mIndexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndexBuffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

		// Create the vertex array object
		glGenVertexArrays(1, &mVertexArrayObject);
		glBindVertexArray(mVertexArrayObject);

		glVertexAttribPointer(VertexAttributePosition, 4, GL_FLOAT, GL_FALSE, sizeof(VertexPositionColor), (void*)offsetof(VertexPositionColor, Position));
		glEnableVertexAttribArray(VertexAttributePosition);

		glVertexAttribPointer(VertexAttributeColor, 4, GL_FLOAT, GL_FALSE, sizeof(VertexPositionColor), (void*)offsetof(VertexPositionColor, Color));
		glEnableVertexAttribArray(VertexAttributeColor);

		glBindVertexArray(0);

		mWorldViewProjectionLocation = glGetUniformLocation(mShaderProgram.Program(), "WorldViewProjection");
		if (mWorldViewProjectionLocation == -1)
		{
			throw GameException("glGetUniformLocation() did not find uniform location.");
		}

		//set the stencils clear value
	}

	void CubeDemo::Update(const GameTime& gameTime)
	{
		//static float angle = 0.0f;

		//angle += static_cast<float>(gameTime.ElapsedGameTime()) * RotationRate;

		//mWorldMatrix = glm::rotate(glm::mat4(), angle, Vector3Helper::Up);
	}

	void CubeDemo::Draw(const GameTime& gameTime)
	{
		//glClear(GL_COLOR_BUFFER_BIT || GL_DEPTH_BUFFER_BIT);

		//glStencilFunc(GL_EQUAL, 0x1, 0x1);
		//glStencilOp(GL_KEEP,  GL_KEEP, GL_KEEP);




		glBindVertexArray(mVertexArrayObject);
		glBindBuffer(GL_ARRAY_BUFFER, mVertexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndexBuffer);

		glUseProgram(mShaderProgram.Program());

		mat4 wvp = mCamera->ViewProjectionMatrix() * mWorldMatrix;
		glUniformMatrix4fv(mWorldViewProjectionLocation, 1, GL_FALSE, &wvp[0][0]);

		glEnable(GL_CULL_FACE);
		glFrontFace(GL_CCW);

		glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
		glPointSize(80.0f);
		//glDrawElements(GL_POINT, 36, GL_UNSIGNED_INT, 0);
		glBindVertexArray(0);
	}
}