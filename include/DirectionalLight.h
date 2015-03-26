//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		DirectionalLight.h
//\author	William Nguatem
//\note		Copyright (C) 
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26 
//******************************************************************************/


#ifndef _Directional_Light_
#define _Directional_Light_

#include "Common.h"
#include "Light.h"

namespace Library
{
	class DirectionalLight : public Light
	{
		//RTTI_DECLARATIONS(DirectionalLight, Light)

	public:
		DirectionalLight(Game& game);
		virtual ~DirectionalLight();

		const glm::vec3& Direction() const;
		const glm::vec3& Up() const;
		const glm::vec3& Right() const;

        void ApplyRotation(const glm::mat4& transform);

	protected:
		glm::vec3 mDirection;
		glm::vec3 mUp;
		glm::vec3 mRight;
	};
}
#endif//_Directional_Light_

