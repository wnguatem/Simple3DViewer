//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		Quads.h
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
    class Quads : public DrawableGameComponent
    {
        //RTTI_DECLARATIONS(ModelDemo, DrawableGameComponent)
        
    public:
        Quads(Game& game, Camera& camera);
        ~Quads();
        
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
        
        Quads();
        Quads(const Quads& rhs);
        Quads& operator=(const Quads& rhs);
        
        void CreateVertexBuffer(const Mesh& mesh, GLuint& vertexBuffer);
        void CreateVertexBuffer(const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud, GLuint& vertexBuffer);
        
        ShaderProgram mShaderProgram;
        GLuint mVertexArrayObject;
        GLuint mVertexBuffer;
        GLuint mIndexBuffer;
        GLint mWorldViewProjectionLocation;
        glm::mat4 mWorldMatrix;
        GLuint mIndexCount;
        
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr m_cloud;
        std::string m_input_data;
    };
}
#endif//_Quad_H_
