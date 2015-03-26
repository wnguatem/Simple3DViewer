//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		KMLModeler.h
//\author	William Nguatem
//\note		Copyright (C) 
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26 
//******************************************************************************/


#ifndef _KML_Modeler_H_
#define _KML_Modeler_H_

#include "DrawableGameComponent.h"
#include "BasicEffect.h"

#include "Model.h"
#include "Mesh.h"

namespace Library
{
	class Mesh;
}

using namespace Library;

namespace Rendering
{
	class KMLModeler : public DrawableGameComponent
	{

	public:
		KMLModeler(Game& game, Camera& camera);

		~KMLModeler();

		virtual void Initialize() override;
		virtual void Draw(const GameTime& gameTime) override;

	private:
		enum VertexAttribute
		{
			VertexAttributePosition = 0,
			VertexAttributeColor = 1
		};

		KMLModeler();
		KMLModeler(const KMLModeler& rhs);
		KMLModeler& operator=(const KMLModeler& rhs);

		void CreateVertexBuffer(const Mesh& mesh, GLuint& vertexBuffer);

		void CreateVertexBuffer(const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud, GLuint& vertexBuffer);

		//user specified color of the point clouds
		void CreateVertexBuffer(const pcl::PointCloud<pcl::PointXYZ>& cloud, const glm::vec4 &vertex_color, GLuint& vertexBuffer);

		//user specified color from uv provided by images, every forth point is a quad geometry
		void CreateVertexBuffer(const pcl::PointCloud<pcl::PointXYZ>& cloud, const std::vector<std::string> &image_names, GLuint& vertexBuffer);

		static const glm::vec4 DefaultColor;

		//ShaderProgram mShaderProgram;
		GLuint mVertexArrayObject;
		GLuint mVertexBufferPnt;
		GLuint mAxesBufferObject;
		GLuint mIndexBuffer;
		GLint mWorldViewProjectionLocation;
		glm::mat4 mWorldMatrix;
		GLuint mIndexCount;
		glm::vec4 mColor;

		BasicEffect mShaderProgram;
		std::vector<GLuint> mVertexBufferPntStrip;
		std::vector<GLuint> mIndexBufferStrip;
		std::vector<GLuint> mIndexCountStrip;
		std::vector<GLuint> mVertexArrayObjectstrip;
		//BasicEffect mShaderProgramStrip;
		std::vector<GLuint> mVertexBuffer;
		std::vector<unsigned int> mVertexCountStrip;
		int mNrStrips;

		std::vector<GLuint> mVertexBufferPntPoly;
		std::vector<GLuint> mIndexBufferPoly;
		std::vector<GLuint> mIndexCountPoly;
		std::vector<GLuint> mVertexArrayObjectPoly;
		//BasicEffect mShaderProgramStrip;
		std::vector<GLuint> mVertexBufferPoly;
		std::vector<unsigned int> mVertexCountPoly;
		int mNrPoly;

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr m_cloud;
		int mShapeType;
	};
}
#endif//_KML_Modeler_H_
