#include "DirectionalLight.h"
#include "VectorHelper.h"

using namespace glm;

namespace Library
{
	//RTTI_DEFINITIONS(DirectionalLight)

	DirectionalLight::DirectionalLight(Game& game)
		: Light(game), mDirection(Vector3Helper::Forward), mUp(Vector3Helper::Up), mRight(Vector3Helper::Right)
	{
	}

	DirectionalLight::~DirectionalLight()
	{
	}

	const vec3& DirectionalLight::Direction() const
	{
		return mDirection;
	}
	
	const vec3& DirectionalLight::Up() const
	{
		return mUp;
	}

	const vec3& DirectionalLight::Right() const
	{
		return mRight;
	}

    void DirectionalLight::ApplyRotation(const mat4& transform)
    {
		vec4 direction = transform * vec4(mDirection, 0.0f);
		mDirection = (vec3)normalize(direction);

		vec4 up = transform * vec4(mUp, 0.0f);
		mUp = (vec3)normalize(up);

		mRight = cross(mDirection, mUp);
		mUp = cross(mRight, mDirection);
    }
}