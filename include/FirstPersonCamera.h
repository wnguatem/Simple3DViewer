//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		FirstPersonCamera.h
//\author	William Nguatem
//\note		Copyright (C) 
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26 
//******************************************************************************/


#ifndef _First_Person_Camera_H_
#define _First_Person_Camera_H_

#include "Camera.h"
#include "Game.h"

namespace Library
{
    class FirstPersonCamera : public Camera
    {
        //RTTI_DECLARATIONS(FirstPersonCamera, Camera)

    public:
        FirstPersonCamera(Game& game);
        FirstPersonCamera(Game& game, float fieldOfView, float aspectRatio, float nearPlaneDistance, float farPlaneDistance);

        virtual ~FirstPersonCamera();

        float& MouseSensitivity();
        float& RotationRate();
        float& MovementRate();		
        
        virtual void Initialize() override;
        virtual void Update(const GameTime& gameTime) override;

        static const float DefaultMouseSensitivity;
        static const float DefaultRotationRate;
        static const float DefaultMovementRate;        

    protected:
        float mMouseSensitivity;
        float mRotationRate;
        float mMovementRate;

    private:
        FirstPersonCamera(const FirstPersonCamera& rhs);
        FirstPersonCamera& operator=(const FirstPersonCamera& rhs);

		double mLastCursorX;
		double mLastCursorY;
    };
}
#endif//_First_Person_Camera_H_
