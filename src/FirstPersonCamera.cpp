#include "FirstPersonCamera.h"
#include "Game.h"
#include "GameTime.h"
#include "VectorHelper.h"

using namespace glm;

namespace Library
{
    //RTTI_DEFINITIONS(FirstPersonCamera)

	const float FirstPersonCamera::DefaultRotationRate = 0.050f;
	const float FirstPersonCamera::DefaultMovementRate = 0.25;// 1.5;// 5.0;// 10.0f;
	const float FirstPersonCamera::DefaultMouseSensitivity = 0.20;//s 1.0f;

    FirstPersonCamera::FirstPersonCamera(Game& game)
        : Camera(game),
          mMouseSensitivity(DefaultMouseSensitivity), mRotationRate(DefaultRotationRate), mMovementRate(DefaultMovementRate),
		  mLastCursorX(0.0), mLastCursorY(0.0)
    {
    }

    FirstPersonCamera::FirstPersonCamera(Game& game, float fieldOfView, float aspectRatio, float nearPlaneDistance, float farPlaneDistance)
        : Camera(game, fieldOfView, aspectRatio, nearPlaneDistance, farPlaneDistance),
          mMouseSensitivity(DefaultMouseSensitivity), mRotationRate(DefaultRotationRate), mMovementRate(DefaultMovementRate),
		  mLastCursorX(0.0), mLastCursorY(0.0)
    {
    }

    FirstPersonCamera::~FirstPersonCamera()
    {
    }

    float&FirstPersonCamera:: MouseSensitivity()
    {
        return mMouseSensitivity;
    }


    float& FirstPersonCamera::RotationRate()
    {
        return mRotationRate;
    }

    float& FirstPersonCamera::MovementRate()
    {
        return mMovementRate;
    }

    void FirstPersonCamera::Initialize()
    {
        Camera::Initialize();

		glfwGetCursorPos(mGame->Window(), &mLastCursorX, &mLastCursorY);
    }

    void FirstPersonCamera::Update(const GameTime& gameTime)
    {
		vec2 movementAmount = Vector2Helper::Zero;
		if (glfwGetKey(mGame->Window(), GLFW_KEY_W))
		{
			//movementAmount.y = 1.0f;
			if (glfwGetKey(mGame->Window(), GLFW_KEY_LEFT_SHIFT))
			{
				movementAmount.y = -1.0f;
			}
			else
			{
				movementAmount.y = -0.20f;
			}
		}

		if (glfwGetKey(mGame->Window(), GLFW_KEY_S))
		{
			//movementAmount.y = -1.0f;
			if (glfwGetKey(mGame->Window(), GLFW_KEY_LEFT_SHIFT))
			{
				movementAmount.y = -1.0f;
			}
			else
			{
				movementAmount.y = -0.20f;
			}
		}

		if (glfwGetKey(mGame->Window(), GLFW_KEY_A))
		{
			if (glfwGetKey(mGame->Window(), GLFW_KEY_LEFT_SHIFT))
			{
				movementAmount.x = -1.0f;
			}
			else
			{
				movementAmount.x = -0.20f;
			}
		}

		if (glfwGetKey(mGame->Window(), GLFW_KEY_D))
		{
			//movementAmount.x = 1.0f;
			if (glfwGetKey(mGame->Window(), GLFW_KEY_LEFT_SHIFT))
			{
				movementAmount.x = -1.0f;
			}
			else
			{
				movementAmount.x = -0.20f;
			}
		}

		if (glfwGetKey(mGame->Window(), GLFW_KEY_Q))
		{
			mPosition.z += 0.5f;
		}

		if (glfwGetKey(mGame->Window(), GLFW_KEY_E))
		{
			mPosition.z -= 0.5f;
		}

		if (glfwGetKey(mGame->Window(), GLFW_KEY_UP))
		{
			mPosition.y += 0.5f;
		}

		if (glfwGetKey(mGame->Window(), GLFW_KEY_DOWN))
		{
			mPosition.y -= 0.5f;
		}

		if (glfwGetKey(mGame->Window(), GLFW_KEY_LEFT))
		{
			mPosition.x += 0.5f;
		}

		if (glfwGetKey(mGame->Window(), GLFW_KEY_RIGHT))
		{
			mPosition.x -= 0.5f;
		}


		vec2 rotationAmount = Vector2Helper::Zero;
		
		double x, y;
		glfwGetCursorPos(mGame->Window(), &x, &y);		

		if (glfwGetMouseButton(mGame->Window(), GLFW_MOUSE_BUTTON_LEFT))
		{
			rotationAmount.x = static_cast<float>(mLastCursorX - x) * mMouseSensitivity;
			rotationAmount.y = static_cast<float>(mLastCursorY - y) * mMouseSensitivity;
		}

		mLastCursorX = x;
		mLastCursorY = y;

		float elapsedTime = (float)gameTime.ElapsedGameTime();
		vec2 rotationVector = rotationAmount * mRotationRate * elapsedTime;

		mat4 rotationMatrix = rotate(mat4(), rotationVector.y, mRight);
		rotationMatrix = rotate(rotationMatrix, rotationVector.x, Vector3Helper::Up);
		ApplyRotation(rotationMatrix);

		vec2 movement = movementAmount * mMovementRate * elapsedTime;

		vec3 strafe = mRight * movement.x;
		mPosition += strafe;

		vec3 forward = mDirection * movement.y;
		mPosition += forward;

        Camera::Update(gameTime);
    }
}
