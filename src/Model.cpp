#include "Model.h"
#include "Game.h"
#include "GameException.h"
#include "Mesh.h"
#include "ModelMaterial.h"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

namespace Library
{
    Model::Model(){}
    
    Model::Model(Game& game, const std::string& filename, bool flipUVs)
		: mGame(game), mMeshes(), mMaterials()
    {
        Assimp::Importer importer;

		unsigned int flags = aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType;
        if (flipUVs)
        {
            flags |= aiProcess_FlipUVs;
        }

        const aiScene* scene = importer.ReadFile(filename, flags);
        if (scene == nullptr)
        {
            throw GameException(importer.GetErrorString());
        }

        if (scene->HasMaterials())
        {
            for (unsigned int i = 0; i < scene->mNumMaterials; i++)
            {
                mMaterials.push_back(new ModelMaterial(*this, scene->mMaterials[i]));
            }
        }

        if (scene->HasMeshes())
        {
            for (unsigned int i = 0; i < scene->mNumMeshes; i++)
            {	
				Mesh* mesh = new Mesh(*this, *(scene->mMeshes[i]));
                mMeshes.push_back(mesh);
            }
        }
	}

	Model::Model(Game& game, const std::string& filename, int diff_d)
		: mGame(game)
	{
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
		//int rrr = pcl::io::loadPCDFile(filename, *cloud);
        // Fill in the cloud data
        pcl::PCDReader reader;
        // Replace the path below with the path where you saved your file
        reader.read (filename, *cloud);
	}

	void Model::GetCloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud)
	{
		cloud = m_cloud;
	}
	
    Model::~Model()
    {
        for (Mesh* mesh : mMeshes)
        {
            delete mesh;
        }

        for (ModelMaterial* material : mMaterials)
        {
            delete material;
        }
    }

    Game& Model::GetGame()
    {
        return mGame;
    }

    bool Model::HasMeshes() const
    {
        return (mMeshes.size() > 0);
    }

    bool Model::HasMaterials() const
    {
        return (mMaterials.size() > 0);
    }

    const std::vector<Mesh*>& Model::Meshes() const
    {
        return mMeshes;
    }

    const std::vector<ModelMaterial*>& Model::Materials() const
    {
        return mMaterials;
    }
}
