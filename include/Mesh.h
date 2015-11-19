//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		Mesh.h
//\author	William Nguatem
//\note		Copyright (C) 
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26 
//******************************************************************************/


#ifndef _Mesh_H_
#define _Mesh_H_

#include "Common.h"

struct aiMesh;

namespace Library
{
    class Material;
    class ModelMaterial;
    class Model;

    class Mesh
    {
        friend class Model;
        friend class ModelMaterial;
        Mesh();
    public:
        ~Mesh();

        Model& GetModel();
        ModelMaterial* GetMaterial();
        const std::string& Name() const;

        const std::vector<glm::vec3>& Vertices() const;
        const std::vector<glm::vec3>& Normals() const;
        const std::vector<glm::vec3>& Tangents() const;
        const std::vector<glm::vec3>& BiNormals() const;
        const std::vector<std::vector<glm::vec3>*>& TextureCoordinates() const;
        const std::vector<std::vector<glm::vec4>*>& VertexColors() const;
        unsigned int FaceCount() const;
        const std::vector<unsigned int>& Indices() const;

        void CreateIndexBuffer(GLuint& indexBuffer);
         Mesh& operator=(const Mesh& rhs);

    private:
        //Mesh() : mModel(nullptr) {}
        Mesh(Model& model, aiMesh& mesh);
        Mesh(const Mesh& rhs);
        //Mesh& operator=(const Mesh& rhs);

        Model& mModel;
        ModelMaterial* mMaterial;
        std::string mName;
        std::vector<glm::vec3> mVertices;
        std::vector<glm::vec3> mNormals;
        std::vector<glm::vec3> mTangents;
        std::vector<glm::vec3> mBiNormals;
        std::vector<std::vector<glm::vec3>*> mTextureCoordinates;
        std::vector<std::vector<glm::vec4>*> mVertexColors;
        unsigned int mFaceCount;
        std::vector<unsigned int> mIndices;
    };
}
#endif//_Mesh_H_