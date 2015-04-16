//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		Quad.h
//\author	William Nguatem
//\note		Copyright (C)
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26
//******************************************************************************/


#ifndef _Quad_H_
#define _Quad_H_

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
    class Quad : public DrawableGameComponent
    {
        //RTTI_DECLARATIONS(ModelDemo, DrawableGameComponent)
        
    public:
        Quad(Game& game, Camera& camera);
        Quad(Game& game, Camera& camera, QuadType &quad, glm::vec4 &color);
        Quad(Game& game, Camera& camera, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const glm::vec4 &color);
        ~Quad();
        
        virtual void Initialize() override;
        virtual void Draw(const GameTime& gameTime) override;
        
        //virtual void setInputData(const std::string &input_data) override;
        void setInputData(const std::string &input_data);
        
    private:
        enum VertexAttribute
        {
            VertexAttributePosition = 0,
            VertexAttributeColor = 1
        };
        
        Quad();
        Quad(const Quad& rhs);
        Quad& operator=(const Quad& rhs);
        
        void CreateVertexBuffer(const Mesh& mesh, GLuint& vertexBuffer);
        void CreateVertexBuffer(const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud, GLuint& vertexBuffer);
        void CreateVertexBuffer(GLuint& vertexBuffer);
        
        ShaderProgram mShaderProgram;
        GLuint mVertexArrayObject;
        GLuint mVertexBuffer;
        GLuint mIndexBuffer;
        GLint mWorldViewProjectionLocation;
        glm::mat4 mWorldMatrix;
        GLuint mIndexCount;
        
        glm::vec4 m_color;
        QuadType m_quad;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
        std::string m_input_data;
    };
}
#endif//_Quad_H_


