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
    
}

namespace Rendering
{
	class CubeDemo;
	class ModelDemo;
    class QuadModeler;
	class KMLModeler;

	class RenderingGame : public Game
	{
		//RTTI_DECLARATIONS(RenderingGame, Game)

	public:
		RenderingGame(const std::wstring& windowTitle);

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
	};
}
#endif//_Rendering_Game_H