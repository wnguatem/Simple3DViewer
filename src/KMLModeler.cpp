//#include "KMLModeler.h"
//#include "Game.h"
//#include "GameException.h"
//#include "ColorHelper.h"
//#include "Camera.h"
//#include "Utility.h"
//#include "ShaderProgram.h"
//#include "VertexDeclarations.h"
//#include "VectorHelper.h"
//#include "Model.h"
//#include "Mesh.h"
//#include "shapefil.h"
//
//#include <pcl/ModelCoefficients.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/project_inliers.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/surface/concave_hull.h>
//#include <pcl/common/transforms.h>
//#include <pcl/features/feature.h>
//#include <pcl/features/normal_3d.h>
//
//#include "kml/dom.h"
//
//
//using namespace glm;
//
//namespace Rendering
//{
//	const vec4 KMLModeler::DefaultColor = vec4(0.961f, 0.871f, 0.702f, 1.0f);
//	KMLModeler::KMLModeler(Game& game, Camera& camera)
//		: DrawableGameComponent(game, camera), mShaderProgram(), mVertexArrayObject(0), mVertexBuffer(0),
//		mIndexBuffer(0), mWorldViewProjectionLocation(-1), mWorldMatrix(), mIndexCount(), mShapeType(0), mColor(DefaultColor)
//	{
//	}
//
//	KMLModeler::~KMLModeler()
//	{
//
//		glDeleteBuffers(1, &mIndexBuffer);
//		glDeleteBuffers(1, &mVertexBufferPnt);
//		for (int i = 0; i < mNrStrips; i++)
//		{
//			glDeleteBuffers(1, &mVertexBuffer.at(i));
//		}
//		glDeleteVertexArrays(1, &mVertexArrayObject);
//	}
//
//	void KMLModeler::Initialize()
//	{
//
//		const float axesPositions[] = {
//			-1.0f, 0.0f, 0.0f, 1.0f,
//			1.0f, 0.0f, 0.0f, 1.0f,
//			0.0f, -1.0, 0.0f, 1.0f,
//			0.0f, 1.0f, 0.0f, 1.0f,
//			0.0f, 0.0f, -1.0f, 1.0f,
//			0.0f, 0.0f, 1.0f, 1.0f,
//		};
//
//		//generate axes vertex buffer
//		GLuint axesBufferObject;
//		glGenBuffers(1, &axesBufferObject);
//
//		glBindBuffer(GL_ARRAY_BUFFER, axesBufferObject);
//		glBufferData(GL_ARRAY_BUFFER, sizeof(axesPositions), axesPositions, GL_STATIC_DRAW);
//		glBindBuffer(GL_ARRAY_BUFFER, 0);
//		mAxesBufferObject = axesBufferObject;
//
//		//SetCurrentDirectory(Utility::ExecutableDirectory().c_str());
//
//		// Build the shader program
//		std::vector<ShaderDefinition> shaders;
//		shaders.push_back(ShaderDefinition(GL_VERTEX_SHADER, "ModelDemo.vert"));
//		shaders.push_back(ShaderDefinition(GL_FRAGMENT_SHADER, "ModelDemo.frag"));
//		mShaderProgram.BuildProgram(shaders);
//
//		//SHPHandle hSHP = SHPOpen("C:\\williamnguatem\\Projects\\ReadEsriShapeFiles\\shape_eg_data\\mpatch3.shp", "rb");
//		//SHPHandle hSHP = SHPWriteObject()
//		SHPHandle hSHP = SHPOpen("C:\\williamnguatem\\Projects\\ReadEsriShapeFiles\\GLRenderSHP_Demo\\GLRenderSHP_Demo\\Shapefiles\\strassen.shp", "rb");
//		//SHPHandle hSHP = SHPOpen("C:\\williamnguatem\\Projects\\ReadEsriShapeFiles\\GLRenderSHP_Demo\\GLRenderSHP_Demo\\Shapefiles\\poi.shp", "rb");
//		//C:\williamnguatem\Projects\ReadEsriShapeFiles\
//
//		if (hSHP == NULL) return;
//
//		mShapeType = hSHP->nShapeType;
//		//Point Shapefile
//		if (mShapeType == SHPT_POINT)
//		{
//			//convert esri models to pcl in other to use pcl functions
//			SHPObject *psShape;
//			size_t nr_points = hSHP->nRecords;
//			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//			cloud->width = nr_points;
//			cloud->height = 1;
//			cloud->resize(nr_points);
//			for (size_t i = 0; i < nr_points; i++)
//			{
//				psShape = SHPReadObject(hSHP, i);
//
//				float fX = psShape->padfX[0];
//				float fY = -psShape->padfY[0];
//				float fZ = psShape->padfZ[0];
//
//				//collect the points
//				cloud->points[i].x = fX;
//				cloud->points[i].y = fY;
//				cloud->points[i].z = fZ;
//			}
//
//			Eigen::Vector3f up_dir(0, 0, 1);// = cloud->points[1582336].getArray3fMap() - cloud->points[3009199].getArray3fMap();
//			up_dir.normalize();
//
//			//flip vp to normalized up direction
//			pcl::PointXYZ vp_up_dir;
//			vp_up_dir.x = up_dir(0);
//			vp_up_dir.y = up_dir(1);
//			vp_up_dir.z = up_dir(2);
//			Eigen::Vector4f fliped_up;
//			//pcl::flipNormalTowardsViewpoint(vp_up_dir, 0, 1, 0, fliped_up);
//			fliped_up.head<3>() = up_dir;
//
//			Eigen::Vector4f cloud_centroid;
//			pcl::compute3DCentroid(*cloud, cloud_centroid);
//			Eigen::Vector3f y_direction(1, 0, 0);
//			Eigen::Vector3f origin_c(cloud_centroid(0), cloud_centroid(1), cloud_centroid(2));
//			Eigen::Affine3f box_transform;
//			pcl::getTransformationFromTwoUnitVectorsAndOrigin(Eigen::Vector3f(fliped_up(0), fliped_up(1), fliped_up(2)), y_direction, origin_c, box_transform);
//
//			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
//			pcl::transformPointCloud(*cloud, *cloud_transformed, box_transform);
//
//			cloud = cloud_transformed;
//
//			//ground
//			int ind_gr;
//			float min_gr = 0.0f;
//
//			//create indices
//			std::vector<unsigned int> cloud_indices(cloud->points.size());
//			for (size_t i = 0; i < cloud->points.size(); i++)
//			{
//				cloud_indices[i] = i;
//				if (cloud->points[i].y < min_gr)
//				{
//					min_gr = cloud->points[i].y;
//					ind_gr = i;
//				}
//			}
//
//			GLuint indexBuffer;
//			glGenBuffers(1, &indexBuffer);
//			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
//			glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * cloud_indices.size(), &cloud_indices[0], GL_STATIC_DRAW);
//			mIndexBuffer = indexBuffer;
//			mIndexCount = cloud_indices.size();
//
//			glm::vec4 vertex_color(0.0f, 1.0f, 0.0f, 1.0f);
//
//			CreateVertexBuffer(*cloud, vertex_color, mVertexBufferPnt);
//			//mesh->CreateIndexBuffer(mIndexBuffer);
//			//mIndexCount = mesh->Indices().size();
//
//			glGenVertexArrays(1, &mVertexArrayObject);
//			glBindVertexArray(mVertexArrayObject);
//
//			glVertexAttribPointer(VertexAttributePosition, 4, GL_FLOAT, GL_FALSE, sizeof(VertexPositionColor), (void*)offsetof(VertexPositionColor, Position));
//			glEnableVertexAttribArray(VertexAttributePosition);
//
//			glVertexAttribPointer(VertexAttributeColor, 4, GL_FLOAT, GL_FALSE, sizeof(VertexPositionColor), (void*)offsetof(VertexPositionColor, Color));
//			glEnableVertexAttribArray(VertexAttributeColor);
//
//			glBindVertexArray(0);
//
//			mWorldViewProjectionLocation = glGetUniformLocation(mShaderProgram.Program(), "WorldViewProjection");
//			if (mWorldViewProjectionLocation == -1)
//			{
//				throw GameException("glGetUniformLocation() did not find uniform location.");
//			}
//		}
//
//		//Line Shapefile
//		if (mShapeType == SHPT_ARC)
//		{
//			SHPObject *psShape;
//			mNrStrips = hSHP->nRecords;
//
//			Eigen::Vector4f cloud_centroid_f(0, 0, 0, 0);
//			for (int j = 0; j < mNrStrips; j++)
//			{
//				psShape = SHPReadObject(hSHP, j);
//				size_t nr_vertices = psShape->nVertices;
//
//				//convert esri models to pcl in other to use pcl functions
//				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//				cloud->width = nr_vertices;
//				cloud->height = 1;
//				cloud->resize(nr_vertices);
//				for (size_t i = 0; i < nr_vertices; i++)
//				{
//					float fX = psShape->padfX[i];
//					float fY = psShape->padfY[i];
//					float fZ = psShape->padfZ[i];
//
//					//collect the points
//					cloud->points[i].x = fX;
//					cloud->points[i].y = fY;
//					cloud->points[i].z = fZ;
//				}
//
//				Eigen::Vector4f cloud_centroid;
//				pcl::compute3DCentroid(*cloud, cloud_centroid);
//				cloud_centroid_f = cloud_centroid_f + cloud_centroid;
//			}
//			cloud_centroid_f(0) = cloud_centroid_f(0) / mNrStrips;
//			cloud_centroid_f(1) = cloud_centroid_f(1) / mNrStrips;
//			cloud_centroid_f(2) = cloud_centroid_f(2) / mNrStrips;
//
//
//			//convert esri models to pcl in other to use pcl functions
//			mVertexArrayObjectstrip.resize(mNrStrips);
//			mIndexBufferStrip.resize(mNrStrips);
//			mIndexCountStrip.resize(mNrStrips);
//			mVertexBufferPntStrip.resize(mNrStrips);
//			mVertexCountStrip.resize(mNrStrips);
//			for (int j = 0; j < mNrStrips; j++)
//			{
//				glGenVertexArrays(1, &mVertexArrayObjectstrip.at(j));
//				glBindVertexArray(mVertexArrayObjectstrip.at(j));
//
//				psShape = SHPReadObject(hSHP, j);
//				size_t nr_vertices = psShape->nVertices;
//				mVertexCountStrip.at(j) = nr_vertices;
//
//
//				//convert esri models to pcl in other to use pcl functions
//				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//				cloud->width = mVertexCountStrip.at(j);
//				cloud->height = 1;
//				cloud->resize(mVertexCountStrip.at(j));
//				for (size_t i = 0; i < mVertexCountStrip.at(j); i++)
//				{
//					float fX = psShape->padfX[i];
//					float fY = psShape->padfY[i];
//					float fZ = psShape->padfZ[i];
//
//					//collect the points
//					cloud->points[i].x = fX;
//					cloud->points[i].y = fY;
//					cloud->points[i].z = fZ;
//				}
//
//				Eigen::Vector3f up_dir(0, 0, 1);// = cloud->points[1582336].getArray3fMap() - cloud->points[3009199].getArray3fMap();
//				up_dir.normalize();
//
//				//flip vp to normalized up direction
//				pcl::PointXYZ vp_up_dir;
//				vp_up_dir.x = up_dir(0);
//				vp_up_dir.y = up_dir(1);
//				vp_up_dir.z = up_dir(2);
//				Eigen::Vector4f fliped_up;
//				//pcl::flipNormalTowardsViewpoint(vp_up_dir, 0, 1, 0, fliped_up);
//				fliped_up.head<3>() = up_dir;
//
//				Eigen::Vector4f cloud_centroid;
//				pcl::compute3DCentroid(*cloud, cloud_centroid);
//				cloud_centroid = cloud_centroid_f;
//				Eigen::Vector3f y_direction(1, 0, 0);
//				Eigen::Vector3f origin_c(cloud_centroid(0), cloud_centroid(1), cloud_centroid(2));
//				Eigen::Affine3f box_transform;
//				pcl::getTransformationFromTwoUnitVectorsAndOrigin(Eigen::Vector3f(fliped_up(0), fliped_up(1), fliped_up(2)), y_direction, origin_c, box_transform);
//
//				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
//				pcl::transformPointCloud(*cloud, *cloud_transformed, box_transform);
//
//				cloud = cloud_transformed;
//
//				//ground
//				int ind_gr;
//				float min_gr = 0.0f;
//
//				//create indices
//				std::vector<unsigned int> cloud_indices(cloud->points.size());
//				for (size_t i = 0; i < cloud->points.size(); i++)
//				{
//					cloud_indices[i] = i;
//					if (cloud->points[i].y < min_gr)
//					{
//						min_gr = cloud->points[i].y;
//						ind_gr = i;
//					}
//				}
//
//				GLuint indexBuffer;
//				glGenBuffers(1, &indexBuffer);
//				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
//				glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * cloud_indices.size(), &cloud_indices[0], GL_STATIC_DRAW);
//				mIndexBufferStrip.at(j) = indexBuffer;
//				mIndexCountStrip.at(j) = cloud_indices.size();
//
//				glm::vec4 vertex_color(0.0f, 1.0f, 0.0f, 1.0f);
//
//				CreateVertexBuffer(*cloud, vertex_color, mVertexBufferPntStrip.at(j));
//				//mesh->CreateIndexBuffer(mIndexBuffer);
//				//mIndexCount = mesh->Indices().size();
//
//				glGenVertexArrays(1, &mVertexArrayObjectstrip.at(j));
//				glBindVertexArray(mVertexArrayObjectstrip.at(j));
//
//				glVertexAttribPointer(VertexAttributePosition, 4, GL_FLOAT, GL_FALSE, sizeof(VertexPositionColor), (void*)offsetof(VertexPositionColor, Position));
//				glEnableVertexAttribArray(VertexAttributePosition);
//
//				glVertexAttribPointer(VertexAttributeColor, 4, GL_FLOAT, GL_FALSE, sizeof(VertexPositionColor), (void*)offsetof(VertexPositionColor, Color));
//				glEnableVertexAttribArray(VertexAttributeColor);
//
//				glBindVertexArray(0);
//
//				mWorldViewProjectionLocation = glGetUniformLocation(mShaderProgram.Program(), "WorldViewProjection");
//				if (mWorldViewProjectionLocation == -1)
//				{
//					throw GameException("glGetUniformLocation() did not find uniform location.");
//				}
//			}
//		}
//
//		//Polygon Shapefile
//		if (mShapeType == SHPT_POLYGON)
//		{
//			SHPObject *psShape;
//			mNrPoly = hSHP->nRecords;
//
//			//get the centroid
//			//convert esri models to pcl in other to use pcl functions
//			Eigen::Vector4f cloud_centroid_f(0, 0, 0, 0);
//			for (int j = 0; j < mNrPoly; j++)
//			{
//				psShape = SHPReadObject(hSHP, j);
//				size_t nr_vertices = psShape->nVertices;
//
//				//convert esri models to pcl in other to use pcl functions
//				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//				cloud->width = nr_vertices;
//				cloud->height = 1;
//				cloud->resize(nr_vertices);
//				for (size_t i = 0; i < nr_vertices; i++)
//				{
//					float fX = psShape->padfX[i];
//					float fY = psShape->padfY[i];
//					float fZ = psShape->padfZ[i];
//
//					//collect the points
//					cloud->points[i].x = fX;
//					cloud->points[i].y = fY;
//					cloud->points[i].z = fZ;
//				}
//
//				Eigen::Vector4f cloud_centroid;
//				pcl::compute3DCentroid(*cloud, cloud_centroid);
//				cloud_centroid_f = cloud_centroid_f + cloud_centroid;
//			}
//			cloud_centroid_f(0) = cloud_centroid_f(0) / mNrPoly;
//			cloud_centroid_f(1) = cloud_centroid_f(1) / mNrPoly;
//			cloud_centroid_f(2) = cloud_centroid_f(2) / mNrPoly;
//
//
//			//convert esri models to pcl in other to use pcl functions
//			mVertexArrayObjectPoly.resize(mNrPoly);
//			mIndexBufferPoly.resize(mNrPoly);
//			mIndexCountPoly.resize(mNrPoly);
//			mVertexBufferPntPoly.resize(mNrPoly);
//			mVertexCountPoly.resize(mNrPoly);
//			for (int j = 0; j < mNrPoly; j++)
//			{
//				glGenVertexArrays(1, &mVertexArrayObjectPoly.at(j));
//				glBindVertexArray(mVertexArrayObjectPoly.at(j));
//
//				psShape = SHPReadObject(hSHP, j);
//				size_t nr_vertices = psShape->nVertices;
//				mVertexCountPoly.at(j) = nr_vertices;
//
//
//				//convert esri models to pcl in other to use pcl functions
//				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//				cloud->width = mVertexCountPoly.at(j);
//				cloud->height = 1;
//				cloud->resize(mVertexCountPoly.at(j));
//				for (size_t i = 0; i < mVertexCountPoly.at(j); i++)
//				{
//					float fX = psShape->padfX[i];
//					float fY = psShape->padfY[i];
//					float fZ = psShape->padfZ[i];
//
//					//collect the points
//					cloud->points[i].x = fX;
//					cloud->points[i].y = fY;
//					cloud->points[i].z = fZ;
//				}
//
//				Eigen::Vector3f up_dir(0, 0, 1);// = cloud->points[1582336].getArray3fMap() - cloud->points[3009199].getArray3fMap();
//				up_dir.normalize();
//
//				//flip vp to normalized up direction
//				pcl::PointXYZ vp_up_dir;
//				vp_up_dir.x = up_dir(0);
//				vp_up_dir.y = up_dir(1);
//				vp_up_dir.z = up_dir(2);
//				Eigen::Vector4f fliped_up;
//				//pcl::flipNormalTowardsViewpoint(vp_up_dir, 0, 1, 0, fliped_up);
//				fliped_up.head<3>() = up_dir;
//
//				Eigen::Vector4f cloud_centroid;
//				pcl::compute3DCentroid(*cloud, cloud_centroid);
//				cloud_centroid = cloud_centroid_f;
//				Eigen::Vector3f y_direction(1, 0, 0);
//				Eigen::Vector3f origin_c(cloud_centroid(0), cloud_centroid(1), cloud_centroid(2));
//				Eigen::Affine3f box_transform;
//				pcl::getTransformationFromTwoUnitVectorsAndOrigin(Eigen::Vector3f(fliped_up(0), fliped_up(1), fliped_up(2)), y_direction, origin_c, box_transform);
//
//				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
//				pcl::transformPointCloud(*cloud, *cloud_transformed, box_transform);
//
//				cloud = cloud_transformed;
//
//				//ground
//				int ind_gr;
//				float min_gr = 0.0f;
//
//				//create indices
//				std::vector<unsigned int> cloud_indices(cloud->points.size());
//				for (size_t i = 0; i < cloud->points.size(); i++)
//				{
//					cloud_indices[i] = i;
//					if (cloud->points[i].y < min_gr)
//					{
//						min_gr = cloud->points[i].y;
//						ind_gr = i;
//					}
//				}
//
//				GLuint indexBuffer;
//				glGenBuffers(1, &indexBuffer);
//				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
//				glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * cloud_indices.size(), &cloud_indices[0], GL_STATIC_DRAW);
//				mIndexBufferPoly.at(j) = indexBuffer;
//				mIndexCountPoly.at(j) = cloud_indices.size();
//
//				glm::vec4 vertex_color(0.0f, 1.0f, 0.0f, 1.0f);
//
//				CreateVertexBuffer(*cloud, vertex_color, mVertexBufferPntPoly.at(j));
//				//mesh->CreateIndexBuffer(mIndexBuffer);
//				//mIndexCount = mesh->Indices().size();
//
//				glGenVertexArrays(1, &mVertexArrayObjectPoly.at(j));
//				glBindVertexArray(mVertexArrayObjectPoly.at(j));
//
//				glVertexAttribPointer(VertexAttributePosition, 4, GL_FLOAT, GL_FALSE, sizeof(VertexPositionColor), (void*)offsetof(VertexPositionColor, Position));
//				glEnableVertexAttribArray(VertexAttributePosition);
//
//				glVertexAttribPointer(VertexAttributeColor, 4, GL_FLOAT, GL_FALSE, sizeof(VertexPositionColor), (void*)offsetof(VertexPositionColor, Color));
//				glEnableVertexAttribArray(VertexAttributeColor);
//
//				glBindVertexArray(0);
//
//				mWorldViewProjectionLocation = glGetUniformLocation(mShaderProgram.Program(), "WorldViewProjection");
//				if (mWorldViewProjectionLocation == -1)
//				{
//					throw GameException("glGetUniformLocation() did not find uniform location.");
//				}
//			}
//		}
//
//
//		// libkml types are in the kmldom namespace
//		using kmldom::CoordinatesPtr;
//		using kmldom::KmlPtr;
//		using kmldom::KmlFactory;
//		using kmldom::PlacemarkPtr;
//		using kmldom::PointPtr;
//
//		// Get the factory singleton to create KML elements.
//		KmlFactory* factory = KmlFactory::GetFactory();
//
//		// Create <coordinates>.
//		CoordinatesPtr coordinates = factory->CreateCoordinates();
//		// Create <coordinates>-122.0816695,37.42052549<coordinates>
//		coordinates->add_latlng(37.42052549, -122.0816695);
//
//		// Create <Point> and give it <coordinates>.
//		PointPtr point = factory->CreatePoint();
//		point->set_coordinates(coordinates);  // point takes ownership
//
//		// Create <Placemark> and give it a <name> and the <Point>.
//		PlacemarkPtr placemark = factory->CreatePlacemark();
//		placemark->set_name("Cool Statue");
//		placemark->set_geometry(point);  // placemark takes ownership
//
//		// Create <kml> and give it <Placemark>.
//		KmlPtr kml = factory->CreateKml();
//		kml->set_feature(placemark);  // kml takes ownership.
//
//		// Serialize to XML
//		std::string xml = kmldom::SerializePretty(kml);
//
//		// Print to stdout
//		std::cout << xml;
//
//
//	}
//
//	void KMLModeler::Draw(const GameTime& gameTime)
//	{
//		if (mShapeType == SHPT_POINT)
//		{
//			glBindVertexArray(mVertexArrayObject);
//			glBindBuffer(GL_ARRAY_BUFFER, mVertexBufferPnt);
//			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndexBuffer);
//
//			mShaderProgram.Use();
//
//			mat4 wvp = mCamera->ViewProjectionMatrix() * mWorldMatrix;
//			glUniformMatrix4fv(mWorldViewProjectionLocation, 1, GL_FALSE, &wvp[0][0]);
//
//			//glEnable(GL_POINT_SPRITE);
//			//glEnable(GL_POINT_SMOOTH);
//			glEnable(GL_BLEND);
//			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//
//			glEnable(GL_CULL_FACE);
//			glFrontFace(GL_CCW);
//
//			////glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
//			//glPointSize(4.0f);
//			//glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
//			////glDrawArrays(GL_QUADS, 0, mIndexCount);
//			////glDrawElements(GL_POINT, mIndexCount, GL_UNSIGNED_INT, 0);
//
//			//glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
//			//glPointSize(3.0f);
//			//glDrawArrays(GL_POINTS, 0, mIndexCount);
//			glDrawArrays(GL_LINE_STRIP, 0, mIndexCount);
//		}
//
//		if (mShapeType == SHPT_ARC)
//		{
//			for (int i = 0; i < mNrStrips; i++)
//			{
//				glBindVertexArray(mVertexArrayObjectstrip.at(i));
//				glBindBuffer(GL_ARRAY_BUFFER, mVertexBufferPntStrip.at(i));
//				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndexBufferStrip.at(i));
//
//				mShaderProgram.Use();
//
//				mat4 wvp = mCamera->ViewProjectionMatrix() * mWorldMatrix;
//				glUniformMatrix4fv(mWorldViewProjectionLocation, 1, GL_FALSE, &wvp[0][0]);
//
//				//glEnable(GL_POINT_SPRITE);
//				//glEnable(GL_POINT_SMOOTH);
//				glEnable(GL_BLEND);
//				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//
//				glEnable(GL_CULL_FACE);
//				glFrontFace(GL_CCW);
//
//				////glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
//				//glPointSize(4.0f);
//				//glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
//				////glDrawArrays(GL_QUADS, 0, mIndexCount);
//				////glDrawElements(GL_POINT, mIndexCount, GL_UNSIGNED_INT, 0);
//
//				//glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
//				//glPointSize(3.0f);
//				//glDrawArrays(GL_POINTS, 0, mIndexCount);
//				glDrawArrays(GL_LINE_STRIP, 0, mIndexCountStrip.at(i));
//			}
//
//		}
//
//		if (mShapeType == SHPT_POLYGON)
//		{
//			for (int i = 0; i < mNrStrips; i++)
//			{
//				glBindVertexArray(mVertexArrayObjectPoly.at(i));
//				glBindBuffer(GL_ARRAY_BUFFER, mVertexBufferPntPoly.at(i));
//				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndexBufferPoly.at(i));
//
//				mShaderProgram.Use();
//
//				mat4 wvp = mCamera->ViewProjectionMatrix() * mWorldMatrix;
//				glUniformMatrix4fv(mWorldViewProjectionLocation, 1, GL_FALSE, &wvp[0][0]);
//
//				//glEnable(GL_POINT_SPRITE);
//				//glEnable(GL_POINT_SMOOTH);
//				glEnable(GL_BLEND);
//				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//
//				glEnable(GL_CULL_FACE);
//				glFrontFace(GL_CCW);
//
//				////glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
//				//glPointSize(4.0f);
//				//glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
//				////glDrawArrays(GL_QUADS, 0, mIndexCount);
//				////glDrawElements(GL_POINT, mIndexCount, GL_UNSIGNED_INT, 0);
//
//				//glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
//				//glPointSize(3.0f);
//				//glDrawArrays(GL_POINTS, 0, mIndexCount);
//				glDrawArrays(GL_LINE_STRIP, 0, mIndexCountStrip.at(i));
//			}
//		}
//		//glDrawElements(GL_POINT, mIndexCount, GL_UNSIGNED_INT, 0);
//
//		//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//		//glClear(GL_COLOR_BUFFER_BIT);
//		//glColor3f(0.0, 0.0, 1.0);
//		//glLoadIdentity();
//
//		////Render Point Shapefile
//		//glColor3f(0.0, 0.0, 1.0);
//		//glEnable(GL_POINT_SMOOTH);
//		//glPointSize(5.0);
//		//glBegin(GL_POINTS);
//
//		//for (int i = 0; i < vPoints.size(); i++)
//		//{
//		//	glVertex2f(vPoints[i].dX, vPoints[i].dY);
//		//}
//
//		//glEnd();
//
//		////Render Line Shapefile
//		//glColor3f(0.0, 1.0, 0.0);
//		//for (i = 0; i < vLines.size(); i++)
//		//{
//
//		//	glBegin(GL_LINE_STRIP);
//		//	for (int j = 0; j < vLines[i].vPointList.size(); j++)
//		//	{
//		//		glVertex2f(vLines[i].vPointList[j].dX, vLines[i].vPointList[j].dY);
//
//		//	}
//
//		//	glEnd();
//		//}
//
//		////Render Polygon Shapefile
//		//glColor3f(1.0, 0.0, 0.0);
//		//for (i = 0; i < vPolygons.size(); i++)
//		//{
//		//	glBegin(GL_LINE_LOOP);
//		//	for (int j = 0; j < vPolygons[i].vPointList.size(); j++)
//		//	{
//		//		glVertex2f(vPolygons[i].vPointList[j].dX, vPolygons[i].vPointList[j].dY);
//		//	}
//
//		//	glEnd();
//		//}
//
//		//glFlush();
//
//		glDisableVertexAttribArray(0);
//
//		//Draw axes
//		glLineWidth(4.0);
//		glBindBuffer(GL_ARRAY_BUFFER, mAxesBufferObject);
//		glEnableVertexAttribArray(0);
//		glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
//
//		glDrawArrays(GL_LINE_STRIP, 0, 2); // x axis
//		glDrawArrays(GL_LINE_STRIP, 2, 2); // y axis
//		glDrawArrays(GL_LINE_STRIP, 4, 2); // z axis
//
//		glDisableVertexAttribArray(0);
//	}
//
//	void KMLModeler::CreateVertexBuffer(const Mesh& mesh, GLuint& vertexBuffer)
//	{
//		const std::vector<vec3>& sourceVertices = mesh.Vertices();
//
//		std::vector<VertexPositionColor> vertices;
//		vertices.reserve(sourceVertices.size());
//		if (mesh.VertexColors().size() > 0)
//		{
//			std::vector<vec4>* vertexColors = mesh.VertexColors().at(0);
//			assert(vertexColors->size() == sourceVertices.size());
//
//			for (unsigned int i = 0; i < sourceVertices.size(); i++)
//			{
//				vec3 position = sourceVertices.at(i);
//				vec4 color = vertexColors->at(i);
//				vertices.push_back(VertexPositionColor(vec4(position.x, position.y, position.z, 1.0f), color));
//			}
//		}
//		else
//		{
//			for (unsigned int i = 0; i < sourceVertices.size(); i++)
//			{
//				vec3 position = sourceVertices.at(i);
//				vec4 color = ColorHelper::RandomColor();
//				vertices.push_back(VertexPositionColor(vec4(position.x, position.y, position.z, 1.0f), color));
//			}
//		}
//
//		glGenBuffers(1, &vertexBuffer);
//		glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
//		glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPositionColor) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
//	}
//
//	void KMLModeler::CreateVertexBuffer(const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud, GLuint& vertexBuffer)
//	{
//		//const std::vector<vec3>& sourceVertices = mesh.Vertices();
//		size_t nr_points = cloud.points.size();
//
//		std::vector<VertexPositionColor> vertices;
//		vertices.reserve(nr_points);
//		if (nr_points > 0)
//		{
//			//std::vector<vec4>* vertexColors = cloud.pointsmesh.VertexColors().at(0);
//			//assert(vertexColors->size() == sourceVertices.size());
//			for (unsigned int i = 0; i < nr_points; i++)
//			{
//				vec3 position = vec3(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
//				//vec4 color = vec4(cloud.points[i].r, cloud.points[i].g, cloud.points[i].b, cloud.points[i].a);
//				vec4 color = ColorHelper::RandomColor();
//				vertices.push_back(VertexPositionColor(vec4(position.x, position.y, position.z, 1.0f), color));
//			}
//		}
//		else
//		{
//			for (unsigned int i = 0; i < nr_points; i++)
//			{
//				vec3 position = vec3(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
//				vec4 color = ColorHelper::RandomColor();
//				vertices.push_back(VertexPositionColor(vec4(position.x, position.y, position.z, 1.0f), color));
//			}
//		}
//
//		glGenBuffers(1, &vertexBuffer);
//		glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
//		glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPositionColor) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
//	}
//
//	//user specified color of the point clouds
//	void KMLModeler::CreateVertexBuffer(const pcl::PointCloud<pcl::PointXYZ>& cloud,
//		const glm::vec4 &vertex_color, GLuint& vertexBuffer)
//	{
//		//const std::vector<vec3>& sourceVertices = mesh.Vertices();
//		size_t nr_points = cloud.points.size();
//
//		std::vector<VertexPositionColor> vertices;
//		vertices.reserve(nr_points);
//		if (nr_points > 0)
//		{
//			//std::vector<vec4>* vertexColors = cloud.pointsmesh.VertexColors().at(0);
//			//assert(vertexColors->size() == sourceVertices.size());
//			for (unsigned int i = 0; i < nr_points; i++)
//			{
//				vec3 position = vec3(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
//				//vec4 color = vec4(cloud.points[i].r, cloud.points[i].g, cloud.points[i].b, cloud.points[i].a);
//				vec4 color = vertex_color;
//				vertices.push_back(VertexPositionColor(vec4(position.x, position.y, position.z, 1.0f), color));
//			}
//		}
//		else
//		{
//			for (unsigned int i = 0; i < nr_points; i++)
//			{
//				vec3 position = vec3(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
//				vec4 color = vertex_color;
//				vertices.push_back(VertexPositionColor(vec4(position.x, position.y, position.z, 1.0f), color));
//			}
//		}
//
//		glGenBuffers(1, &vertexBuffer);
//		glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
//		glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPositionColor) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
//	}
//
//}