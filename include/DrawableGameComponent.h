//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		DrawableGameComponent.h
//\author	William Nguatem
//\note		Copyright (C) 
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26 
//******************************************************************************/


#ifndef _Drawable_Game_Component_H_
#define _Drawable_Game_Component_H_

#include "Common.h"
#include "GameComponent.h"

namespace Library
{
	class Camera;

	class DrawableGameComponent : public GameComponent
	{
		//RTTI_DECLARATIONS(DrawableGameComponent, GameComponent)

	public:
		DrawableGameComponent();
		DrawableGameComponent(Game& game);
		DrawableGameComponent(Game& game, Camera& camera);
		virtual ~DrawableGameComponent();

		bool Visible() const;
		void SetVisible(bool visible);

		Camera* GetCamera();
		void SetCamera(Camera* camera);

		virtual void Draw(const GameTime& gameTime);

	protected:
		bool mVisible;
		Camera* mCamera;

	private:
		DrawableGameComponent(const DrawableGameComponent& rhs);
		DrawableGameComponent& operator=(const DrawableGameComponent& rhs);
	};
}
#endif//_Drawable_Game_Component_H_