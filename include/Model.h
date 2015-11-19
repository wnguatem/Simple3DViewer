//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		Model.h
//\author	William Nguatem
//\note		Copyright (C) 
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26 
//******************************************************************************/


#ifndef _Model_H_
#define _Model_H_

#include "Common.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

struct aiNode;

namespace Library
{
    class Game;
    class Mesh;
    class ModelMaterial;
	class AnimationClip;

    class Model
    {
		friend class Mesh;
        Model();
    public:
        Model(Game& game, const std::string& filename, bool flipUVs = false);
		Model(Game& game, const std::string& filename, int diff_d);
        ~Model();

        Game& GetGame();
        bool HasMeshes() const;
        bool HasMaterials() const;

        const std::vector<Mesh*>& Meshes() const;
        const std::vector<ModelMaterial*>& Materials() const;
		void GetCloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud);

    private:
        Model(const Model& rhs);
        Model& operator=(const Model& rhs);

        Game& mGame;
        std::vector<Mesh*> mMeshes;
        std::vector<ModelMaterial*> mMaterials;

		pcl::PointCloud<pcl::PointXYZRGBNormal> m_cloud;
    };
}
#endif//_Model_H_