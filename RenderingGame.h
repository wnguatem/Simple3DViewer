//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		RenderingGame.h
//\author	William Nguatem
//\note		Copyright (C) 
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26 
//******************************************************************************/


#ifndef _Rendering_Game_H_
#define _Rendering_Game_H_

#include "DrawableGameComponent.h"
#include "BasicEffect.h"

#include "Model.h"
#include "Mesh.h"

#include "Game.h"

using namespace Library;

namespace Library
{
	class GameTime;
	class FirstPersonCamera;
	class Grid;
    class Skybox;
    
}

namespace Rendering
{
	class CubeDemo;
	class ModelDemo;
    class QuadModeler;
	class KMLModeler;
    //class Quad;
    class Quads;
    class TexturedQuad;
    class SpotLightDemo;
    class NormalMappingDemo;
    class TransparencyMappingDemo;
    class EnvironmentMappingDemo;
    class FogDemo;
    class FacadeFileModeler;
	class CubeDemo;
	class TexturedMeshloader;
    

	class RenderingGame : public Game
	{
		//RTTI_DECLARATIONS(RenderingGame, Game)


	public:
		RenderingGame(const std::wstring& windowTitle);
        
        virtual std::string getInputData() override;
        virtual void setInputData(const std::string &input_data) override;

		virtual std::wstring getGameContentFolder() override;
        
        //todo: Refactor this to a class called vector data primitives and use a Rtti to determnig what is to be instatiated
        void addQuad(const glm::vec4 &vertices, const glm::vec3 &color);
        void addQuads(const std::vector<glm::vec4>, const glm::vec3 &color);
        void addQuad(const pcl::PointCloud<pcl::PointXYZ>::Ptr &quad_cloud, const glm::vec4 &color);
        void addQuads(const pcl::PointCloud<pcl::PointXYZ>::Ptr &quad_cloud, const glm::vec3 &color);
        
	protected:
		virtual void Initialize() override;
		virtual void Draw(const GameTime& gameTime) override;
		virtual void Shutdown() override;
        
	private:
		void OnKey(int key, int scancode, int action, int mods);

		FirstPersonCamera* mCamera;
		KeyboardHandler mKeyboardHandler;
		Grid* mGrid;

		CubeDemo* mCubeDemo;
		ModelDemo* mPointDemo;
        QuadModeler* mQuadModel;
		KMLModeler* mKMLModel;
       // Quad* mQuad;
        Quads* mQuads;
        TexturedQuad* mTexturedQuad;
        SpotLightDemo* mSpotLightDemo;
        NormalMappingDemo* mNormalMappingDemo;
        TransparencyMappingDemo* mTransparencyMappingDemo;
        Skybox* mSkybox;
        FogDemo* mFogDemo;
        EnvironmentMappingDemo* mEnvironmentMappingDemo;
        FacadeFileModeler* mFacadeFileModeler;

		TexturedMeshloader* lodgenMesh;
        
        std::string m_input_data;
        pcl::PointCloud<pcl::PointXYZ>::Ptr m_quad_cloud;
        glm::vec4 m_quad_color;


		std::wstring mGameContentFolder;
	};
}
#endif//_Rendering_Game_H