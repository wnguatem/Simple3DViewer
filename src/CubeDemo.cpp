#include "CubeDemo.h"
#include "Game.h"
#include "GameException.h"
#include "ColorHelper.h"
#include "Camera.h"
#include "Utility.h"
#include "ShaderProgram.h"
#include "VertexDeclarations.h"
#include "VectorHelper.h"

#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>



using namespace glm;

namespace Rendering
{
	//RTTI_DEFINITIONS(CubeDemo)

	//const GLfloat CubeDemo::RotationRate = 180.0f;
	const GLfloat CubeDemo::RotationRate = 600.0f;

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
        
#ifdef __APPLE__
        const std::string game_content = "/Users/williamnguatem/projects/LODViewer/game_content";
#endif
        
#ifdef WIN32
        const std::string game_content = "C:/williamnguatem/Projects/Simple3DViewer/game_content";
#endif
        

		// Build the shader program
		std::vector<ShaderDefinition> shaders;
		shaders.push_back(ShaderDefinition(GL_VERTEX_SHADER, game_content+"/CubeDemo.vert"));
		shaders.push_back(ShaderDefinition(GL_FRAGMENT_SHADER, game_content+"/CubeDemo.frag"));
		mShaderProgram.BuildProgram(shaders);


		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::io::loadPCDFile("Q:/DataSet/wind_detector_from4/data100/1/out/predictions/nms_positives/predicted_curve_boundary_0_good_0.pcd", *cloud);

		pcl::PointXYZ minPt, maxPt;
		pcl::getMinMax3D(*cloud, minPt, maxPt);

		
		//// Create the vertex buffer object
		//VertexPositionColor vertices[] =
		//{
		//	VertexPositionColor(vec4(-1.0f, +1.0f, -1.0f, 1.0f), ColorHelper::Green),
		//	VertexPositionColor(vec4(+1.0f, +1.0f, -1.0f, 1.0f), ColorHelper::Yellow),
		//	VertexPositionColor(vec4(+1.0f, +1.0f, +1.0f, 1.0f), ColorHelper::White),
		//	VertexPositionColor(vec4(-1.0f, +1.0f, +1.0f, 1.0f), ColorHelper::BlueGreen),

		//	VertexPositionColor(vec4(-1.0f, -1.0f, +1.0f, 1.0f), ColorHelper::Blue),
		//	VertexPositionColor(vec4(+1.0f, -1.0f, +1.0f, 1.0f), ColorHelper::Purple),
		//	VertexPositionColor(vec4(+1.0f, -1.0f, -1.0f, 1.0f), ColorHelper::Red),
		//	VertexPositionColor(vec4(-1.0f, -1.0f, -1.0f, 1.0f), ColorHelper::Black)
		//};

		// Create the vertex buffer object
		VertexPositionColor vertices[] =
		{
			VertexPositionColor(vec4(minPt.x, maxPt.y, -1.0f, 1.0f), ColorHelper::Green),
			VertexPositionColor(vec4(maxPt.x, maxPt.y, -1.0f, 1.0f), ColorHelper::Yellow),
			VertexPositionColor(vec4(maxPt.x, maxPt.y, +1.0f, 1.0f), ColorHelper::White),
			VertexPositionColor(vec4(minPt.x, maxPt.y, +1.0f, 1.0f), ColorHelper::BlueGreen),

			VertexPositionColor(vec4(minPt.x, minPt.y, +1.0f, 1.0f), ColorHelper::Blue),
			VertexPositionColor(vec4(maxPt.x, minPt.y, +1.0f, 1.0f), ColorHelper::Purple),
			VertexPositionColor(vec4(maxPt.x, minPt.y, -1.0f, 1.0f), ColorHelper::Red),
			VertexPositionColor(vec4(minPt.x, minPt.y, -1.0f, 1.0f), ColorHelper::Black)
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
	}

	void CubeDemo::Update(const GameTime& gameTime)
	{
		static float angle = 0.0f;

		//angle += static_cast<float>(gameTime.ElapsedGameTime()) * RotationRate;

		mWorldMatrix = glm::rotate(glm::mat4(), angle, Vector3Helper::Up);
	}

	void CubeDemo::Draw(const GameTime& gameTime)
	{
		glBindVertexArray(mVertexArrayObject);
		glBindBuffer(GL_ARRAY_BUFFER, mVertexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndexBuffer);

		glUseProgram(mShaderProgram.Program());

		mat4 wvp = mCamera->ViewProjectionMatrix() * mWorldMatrix;
		glUniformMatrix4fv(mWorldViewProjectionLocation, 1, GL_FALSE, &wvp[0][0]);

		glEnable(GL_CULL_FACE);
		glFrontFace(GL_CCW);

		glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
		glBindVertexArray(0);
	}
}