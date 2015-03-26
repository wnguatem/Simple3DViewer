//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		Light.h
//\author	William Nguatem
//\note		Copyright (C) 
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26 
//******************************************************************************/


#ifndef _Light_H_
#define _Lights_H_

#include "Common.h"
#include "GameComponent.h"

namespace Library
{
	class Light : public GameComponent
	{
		//RTTI_DECLARATIONS(Light, GameComponent)

	public:
		Light(Game& game);
		virtual ~Light();

		const glm::vec4& Color() const;
		//void SetColor(FLOAT r, FLOAT g, FLOAT b, FLOAT a);
		void SetColor(float r, float g, float b, float a);
		void SetColor(const glm::vec4& color);

	protected:
		glm::vec4 mColor;
	};
}
#endif//_Lights_H_

