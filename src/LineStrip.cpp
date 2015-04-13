#include "LineStrip.h"
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
#include "shapefil.h"

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


#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>




using namespace glm;

namespace Rendering
{
	const vec4 FacadeFileModeler::DefaultColor = vec4(0.961f, 0.871f, 0.702f, 1.0f);
	FacadeFileModeler::FacadeFileModeler(Game& game, Camera& camera)
		: DrawableGameComponent(game, camera), mShaderProgram(), mVertexArrayObject(0), mVertexBuffer(0),
		mIndexBuffer(0), mWorldViewProjectionLocation(-1), mWorldMatrix(), mIndexCount(), mShapeType(0), mColor(DefaultColor),
    m_input_data(game.getInputData()),mMainFacadeVertexArrayObject(0), mMainFacadeVertexBuffer(0),
    mMainFacadeIndexBuffer(0)
	{
	}

	FacadeFileModeler::~FacadeFileModeler()
	{

		glDeleteBuffers(1, &mIndexBuffer);
		glDeleteBuffers(1, &mVertexBufferPnt);
		for (int i = 0; i < mNrStrips; i++)
		{
			glDeleteBuffers(1, &mVertexBuffer.at(i));
		}
		glDeleteVertexArrays(1, &mVertexArrayObject);
	}


	void FacadeFileModeler::Initialize()
	{

		//Axes in 3D
		//Vertices for the axes in 3 dimesions
		const float axesPositions[] = {
			-1.0f, 0.0f, 0.0f, 1.0f,
			1.0f, 0.0f, 0.0f, 1.0f,
			0.0f, -1.0, 0.0f, 1.0f,
			0.0f, 1.0f, 0.0f, 1.0f,
			0.0f, 0.0f, -1.0f, 1.0f,
			0.0f, 0.0f, 1.0f, 1.0f,
		};

		//generate axes vertex buffer
		GLuint axesBufferObject;
		glGenBuffers(1, &axesBufferObject);

		glBindBuffer(GL_ARRAY_BUFFER, axesBufferObject);
		glBufferData(GL_ARRAY_BUFFER, sizeof(axesPositions), axesPositions, GL_STATIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		mAxesBufferObject = axesBufferObject;

		// Build the shader program
		std::vector<ShaderDefinition> shaders;
		shaders.push_back(ShaderDefinition(GL_VERTEX_SHADER, L"ModelDemo.vert"));
		shaders.push_back(ShaderDefinition(GL_FRAGMENT_SHADER, L"ModelDemo.frag"));
		mShaderProgram.BuildProgram(shaders);

        
        //use the reference cloud to push everything to the correct ground plane
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_0(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::io::loadPCDFile(m_input_data, *cloud_0);
        //force z to point upwards
        for (size_t i = 0; i < cloud_0->points.size(); i++) {
            float tmp_val = cloud_0->points[i].y;
            cloud_0->points[i].y = cloud_0->points[i].z;
            cloud_0->points[i].z = tmp_val;
        }
        
        pcl::PointXYZRGBNormal min_pnt, max_pnt;
        pcl::getMinMax3D(*cloud_0, min_pnt, max_pnt);
        
        
        std::vector <std::string> input_files;
        
        namespace fs = boost::filesystem;
        
        fs::path targetDir("/Volumes/Elements/test_data/plummer-bonn/out/curves");
        
        fs::directory_iterator it(targetDir), eod;
        
        BOOST_FOREACH(fs::path const &p, std::make_pair(it, eod))
        { 
            if(fs::is_regular_file(p) && p.extension().string()== ".pcd")
            {
                input_files.push_back(p.string());
            } 
        }
  
            			//SHPObject *psShape;
            			mNrStrips = input_files.size();
            
            			Eigen::Vector4f cloud_centroid_f(0, 0, 0, 0);
            			for (int j = 0; j < mNrStrips; j++)
            			{
                            
                            //convert esri models to pcl in other to use pcl functions
                            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                            pcl::io::loadPCDFile(input_files.at(j), *cloud);
            				Eigen::Vector4f cloud_centroid;
            				pcl::compute3DCentroid(*cloud, cloud_centroid);
            				cloud_centroid_f = cloud_centroid_f + cloud_centroid;
            			}
            			cloud_centroid_f(0) = cloud_centroid_f(0) / mNrStrips;
            			cloud_centroid_f(1) = cloud_centroid_f(1) / mNrStrips;
            			cloud_centroid_f(2) = cloud_centroid_f(2) / mNrStrips;
            
            
            			//convert esri models to pcl in other to use pcl functions
            			mVertexArrayObjectstrip.resize(mNrStrips);
            			mIndexBufferStrip.resize(mNrStrips);
            			mIndexCountStrip.resize(mNrStrips);
            			mVertexBufferPntStrip.resize(mNrStrips);
            			mVertexCountStrip.resize(mNrStrips);
            			for (int j = 0; j < mNrStrips; j++)
            			{
            				glGenVertexArrays(1, &mVertexArrayObjectstrip.at(j));
            				glBindVertexArray(mVertexArrayObjectstrip.at(j));
                            
                            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                            pcl::io::loadPCDFile(input_files.at(j), *cloud);
            				size_t nr_vertices = cloud->points.size();
                            mVertexCountStrip.at(j) = nr_vertices;
                            
                            //force z to point upwards
                            for (size_t i = 0; i < cloud->points.size(); i++) {
                                float tmp_val = cloud->points[i].y;
                                cloud->points[i].y = cloud->points[i].z - min_pnt.y;
                                cloud->points[i].z = tmp_val;
                            }

                            //Commented this out since we just want but to visualize what is passed in!!!
                            
//            				Eigen::Vector3f up_dir(0, 0, 1);// = cloud->points[1582336].getArray3fMap() - cloud->points[3009199].getArray3fMap();
//            				up_dir.normalize();
//            
//            				//flip vp to normalized up direction
//            				pcl::PointXYZ vp_up_dir;
//            				vp_up_dir.x = up_dir(0);
//            				vp_up_dir.y = up_dir(1);
//            				vp_up_dir.z = up_dir(2);
//            				Eigen::Vector4f fliped_up;
//            				//pcl::flipNormalTowardsViewpoint(vp_up_dir, 0, 1, 0, fliped_up);
//            				fliped_up.head<3>() = up_dir;
//            
//            				Eigen::Vector4f cloud_centroid;
//            				pcl::compute3DCentroid(*cloud, cloud_centroid);
//            				cloud_centroid = cloud_centroid_f;
//            				Eigen::Vector3f y_direction(1, 0, 0);
//            				Eigen::Vector3f origin_c(cloud_centroid(0), cloud_centroid(1), cloud_centroid(2));
//            				Eigen::Affine3f box_transform;
//            				pcl::getTransformationFromTwoUnitVectorsAndOrigin(Eigen::Vector3f(fliped_up(0), fliped_up(1), fliped_up(2)), y_direction, origin_c, box_transform);
//            
//            				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
//            				pcl::transformPointCloud(*cloud, *cloud_transformed, box_transform);
//            
//            				cloud = cloud_transformed;
            
            				//ground
            				int ind_gr;
            				float min_gr = 0.0f;
            
            				//create indices
            				std::vector<unsigned int> cloud_indices(cloud->points.size());
            				for (size_t i = 0; i < cloud->points.size(); i++)
            				{
            					cloud_indices[i] = i;
            					if (cloud->points[i].y < min_gr)
            					{
            						min_gr = cloud->points[i].y;
            						ind_gr = i;
            					}
            				}
            
            				GLuint indexBuffer;
            				glGenBuffers(1, &indexBuffer);
            				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
            				glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * cloud_indices.size(), &cloud_indices[0], GL_STATIC_DRAW);
            				mIndexBufferStrip.at(j) = indexBuffer;
            				mIndexCountStrip.at(j) = cloud_indices.size();
            
            				glm::vec4 vertex_color(0.0f, 1.0f, 0.0f, 1.0f);
            
            				CreateVertexBuffer(*cloud, vertex_color, mVertexBufferPntStrip.at(j));
            				//mesh->CreateIndexBuffer(mIndexBuffer);
            				//mIndexCount = mesh->Indices().size();
            
            				glGenVertexArrays(1, &mVertexArrayObjectstrip.at(j));
            				glBindVertexArray(mVertexArrayObjectstrip.at(j));
            
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
        
        //initialize main facade cloud
        //initMainFacadeCloud();
        
	}
    
    void FacadeFileModeler::initMainFacadeCloud()
    {
        //just use thesame shader program
        // Build the shader program
//        std::vector<ShaderDefinition> shaders;
//        shaders.push_back(ShaderDefinition(GL_VERTEX_SHADER, L"ModelDemo.vert"));
//        shaders.push_back(ShaderDefinition(GL_FRAGMENT_SHADER, L"ModelDemo.frag"));
//        mShaderProgram.BuildProgram(shaders);
        
        // Load the model
        //std::unique_ptr<Model> model(new Model(*mGame, "Sphere.obj"));
        //std::unique_ptr<Model> model(new Model(*mGame, "C:\\williamnguatem\\Projects\\3DViewer\\build\\model_cloud.ply"));
        //std::unique_ptr<Model> model(new Model(*mGame, "C:\\williamnguatem\\Projects\\3DViewer\\build\\bunny\\bunny\\reconstruction\\bun_zipper.ply"));
        
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        
        //std::string filename = m_input_data;
        
        pcl::io::loadPCDFile(m_input_data, *cloud);
        
        for (size_t i = 0; i < cloud->points.size(); i++) {
            float tmp_val = cloud->points[i].y;
            cloud->points[i].y = cloud->points[i].z;
            cloud->points[i].z = tmp_val;
        }
        
        
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
        mMainFacadeIndexBuffer = indexBuffer;
        mMainFacadeIndexCount = cloud_indices.size();
        
        CreateVertexBuffer(*cloud, mMainFacadeVertexBuffer);
        
        glGenVertexArrays(1, &mMainFacadeVertexArrayObject);
        glBindVertexArray(mMainFacadeVertexArrayObject);
        
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

	void FacadeFileModeler::Draw(const GameTime& gameTime)
	{
        //draw the curves
			for (int i = 0; i < mNrStrips; i++)
			{
				glBindVertexArray(mVertexArrayObjectstrip.at(i));
				glBindBuffer(GL_ARRAY_BUFFER, mVertexBufferPntStrip.at(i));
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndexBufferStrip.at(i));

				mShaderProgram.Use();

				mat4 wvp = mCamera->ViewProjectionMatrix() * mWorldMatrix;
				glUniformMatrix4fv(mWorldViewProjectionLocation, 1, GL_FALSE, &wvp[0][0]);

				//glEnable(GL_POINT_SPRITE);
				//glEnable(GL_POINT_SMOOTH);
				glEnable(GL_BLEND);
				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

				glEnable(GL_CULL_FACE);
				glFrontFace(GL_CCW);
				glDrawArrays(GL_LINE_STRIP, 0, mIndexCountStrip.at(i));
			}
        
		glDisableVertexAttribArray(0);
        
        
        //draw the main facade point inliers
        glBindVertexArray(mMainFacadeVertexArrayObject);
        glBindBuffer(GL_ARRAY_BUFFER, mMainFacadeVertexBuffer);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mMainFacadeIndexBuffer);
        
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
        glDrawArrays(GL_POINTS, 0, mMainFacadeIndexCount);
        //glDrawElements(GL_POINTS, mMainFacadeIndexCount, GL_UNSIGNED_INT, 0);
        glDisableVertexAttribArray(0);
        

		//Draw axes
		glLineWidth(4.0);
		glBindBuffer(GL_ARRAY_BUFFER, mAxesBufferObject);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);

		glDrawArrays(GL_LINE_STRIP, 0, 2); // x axis
		glDrawArrays(GL_LINE_STRIP, 2, 2); // y axis
		glDrawArrays(GL_LINE_STRIP, 4, 2); // z axis

		glDisableVertexAttribArray(0);
	}

	void FacadeFileModeler::CreateVertexBuffer(const Mesh& mesh, GLuint& vertexBuffer)
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

	void FacadeFileModeler::CreateVertexBuffer(const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud, GLuint& vertexBuffer)
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

	//user specified color of the point clouds
	void FacadeFileModeler::CreateVertexBuffer(const pcl::PointCloud<pcl::PointXYZ>& cloud,
		const glm::vec4 &vertex_color, GLuint& vertexBuffer)
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
				vec4 color = vertex_color;
				vertices.push_back(VertexPositionColor(vec4(position.x, position.y, position.z, 1.0f), color));
			}
		}
		else
		{
			for (unsigned int i = 0; i < nr_points; i++)
			{
				vec3 position = vec3(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
				vec4 color = vertex_color;
				vertices.push_back(VertexPositionColor(vec4(position.x, position.y, position.z, 1.0f), color));
			}
		}

		glGenBuffers(1, &vertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
		glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPositionColor) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
	}

}