//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		PointLight.h
//\author	William Nguatem
//\note		Copyright (C) 
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26 
//******************************************************************************/


#ifndef _Point_Light_H_
#define _Point_Light_H_

#include "Common.h"
#include "Light.h"

namespace Library
{
	class PointLight : public Light
	{
		//RTTI_DECLARATIONS(PointLight, Light)

	public:
		PointLight(Game& game);
		virtual ~PointLight();

		const glm::vec3& Position() const;
		float Radius() const;

		virtual void SetPosition(float x, float y, float z);
        virtual void SetPosition(const glm::vec3& position);
		virtual void SetRadius(float value);

		static const float DefaultRadius;

	protected:
		glm::vec3 mPosition;
		float mRadius;
	};
}
#endif//_Point_Light