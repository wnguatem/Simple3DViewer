#include "VisPointCloud.h"
#include "Game.h"
#include "GameException.h"
#include "ColorHelper.h"
#include "Camera.h"
#include "Utility.h"
#include "ShaderProgram.h"
#include "VertexDeclarations.h"
#include "VectorHelper.h"
#include "Model.h"
#include "Mesh.h"


using namespace glm;

namespace Rendering
{
	//RTTI_DEFINITIONS(ModelDemo)

		ModelDemo::ModelDemo(Game& game, Camera& camera)
		: DrawableGameComponent(game, camera), mShaderProgram(), mVertexArrayObject(0), mVertexBuffer(0),
		mIndexBuffer(0), mWorldViewProjectionLocation(-1), mWorldMatrix(), mIndexCount()
	{
	}

	ModelDemo::~ModelDemo()
	{
		glDeleteBuffers(1, &mIndexBuffer);
		glDeleteBuffers(1, &mVertexBuffer);
		glDeleteVertexArrays(1, &mVertexArrayObject);
	}

	void ModelDemo::Initialize()
	{
		//SetCurrentDirectory(Utility::ExecutableDirectory().c_str());

		// Build the shader program
		std::vector<ShaderDefinition> shaders;
		shaders.push_back(ShaderDefinition(GL_VERTEX_SHADER, L"ModelDemo.vert"));
		shaders.push_back(ShaderDefinition(GL_FRAGMENT_SHADER, L"ModelDemo.frag"));
		mShaderProgram.BuildProgram(shaders);

		// Load the model
		//std::unique_ptr<Model> model(new Model(*mGame, "Sphere.obj"));
		//std::unique_ptr<Model> model(new Model(*mGame, "C:\\williamnguatem\\Projects\\3DViewer\\build\\model_cloud.ply"));
		//std::unique_ptr<Model> model(new Model(*mGame, "C:\\williamnguatem\\Projects\\3DViewer\\build\\bunny\\bunny\\reconstruction\\bun_zipper.ply"));

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

		pcl::io::loadPCDFile("TestData.pcd", *cloud);

		// Create the vertex and index buffers
		//Mesh* mesh = model->Meshes().at(0);
		//CreateVertexBuffer(*mesh, mVertexBuffer);
		//mesh->CreateIndexBuffer(mIndexBuffer);
		//mIndexCount = mesh->Indices().size();

		//create indices

		std::vector<unsigned int> cloud_indices(cloud->points.size());
		for (size_t i = 0; i < cloud->points.size(); i++)
		{
			cloud_indices[i] = i;
		}

		GLuint indexBuffer;
		glGenBuffers(1, &indexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * cloud_indices.size(), &cloud_indices[0], GL_STATIC_DRAW);
		mIndexBuffer = indexBuffer;
		mIndexCount = cloud_indices.size();

		CreateVertexBuffer(*cloud, mVertexBuffer);
		//mesh->CreateIndexBuffer(mIndexBuffer);
		//mIndexCount = mesh->Indices().size();


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

	void ModelDemo::Draw(const GameTime& gameTime)
	{
		glBindVertexArray(mVertexArrayObject);
		glBindBuffer(GL_ARRAY_BUFFER, mVertexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndexBuffer);

		mShaderProgram.Use();

		mat4 wvp = mCamera->ViewProjectionMatrix() * mWorldMatrix;
		glUniformMatrix4fv(mWorldViewProjectionLocation, 1, GL_FALSE, &wvp[0][0]);

		//glEnable(GL_POINT_SPRITE);
		//GL_POINT_SMOOTH;
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		
		glEnable(GL_CULL_FACE);
		glFrontFace(GL_CCW);

		//glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
		glPointSize(3.0f);
		glDrawArrays(GL_POINTS, 0, mIndexCount);
		//glDrawElements(GL_POINT, mIndexCount, GL_UNSIGNED_INT, 0);
	}

	void ModelDemo::CreateVertexBuffer(const Mesh& mesh, GLuint& vertexBuffer)
	{
		const std::vector<vec3>& sourceVertices = mesh.Vertices();

		std::vector<VertexPositionColor> vertices;
		vertices.reserve(sourceVertices.size());
		if (mesh.VertexColors().size() > 0)
		{
			std::vector<vec4>* vertexColors = mesh.VertexColors().at(0);
			assert(vertexColors->size() == sourceVertices.size());

			for (unsigned int i = 0; i < sourceVertices.size(); i++)
			{
				vec3 position = sourceVertices.at(i);
				vec4 color = vertexColors->at(i);
				vertices.push_back(VertexPositionColor(vec4(position.x, position.y, position.z, 1.0f), color));
			}
		}
		else
		{
			for (unsigned int i = 0; i < sourceVertices.size(); i++)
			{
				vec3 position = sourceVertices.at(i);
				vec4 color = ColorHelper::RandomColor();
				vertices.push_back(VertexPositionColor(vec4(position.x, position.y, position.z, 1.0f), color));
			}
		}

		glGenBuffers(1, &vertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
		glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPositionColor) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
	}

	void ModelDemo::CreateVertexBuffer(const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud, GLuint& vertexBuffer)
	{
		//const std::vector<vec3>& sourceVertices = mesh.Vertices();
		size_t nr_points = cloud.points.size();

		std::vector<VertexPositionColor> vertices;
		vertices.reserve(nr_points);
		if (nr_points > 0)
		{
			//std::vector<vec4>* vertexColors = cloud.pointsmesh.VertexColors().at(0);
			//assert(vertexColors->size() == sourceVertices.size());
			for (unsigned int i = 0; i < nr_points; i++)
			{
				vec3 position = vec3(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
				//vec4 color = vec4(cloud.points[i].r, cloud.points[i].g, cloud.points[i].b, cloud.points[i].a);
				vec4 color = ColorHelper::RandomColor();
				vertices.push_back(VertexPositionColor(vec4(position.x, position.y, position.z, 1.0f), color));
			}
		}
		else
		{
			for (unsigned int i = 0; i < nr_points; i++)
			{
				vec3 position = vec3(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
				vec4 color = ColorHelper::RandomColor();
				vertices.push_back(VertexPositionColor(vec4(position.x, position.y, position.z, 1.0f), color));
			}
		}

		glGenBuffers(1, &vertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
		glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPositionColor) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
	}
}