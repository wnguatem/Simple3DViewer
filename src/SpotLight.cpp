#include "SpotLight.h"
#include "VectorHelper.h"

using namespace glm;

namespace Library
{
	//RTTI_DEFINITIONS(SpotLight)

	const float SpotLight::DefaultInnerAngle = 0.75f;
	const float SpotLight::DefaultOuterAngle = 0.25f;

	SpotLight::SpotLight(Game& game)
		: PointLight(game), mInnerAngle(DefaultInnerAngle), mOuterAngle(DefaultOuterAngle),
		  mDirection(Vector3Helper::Forward), mUp(Vector3Helper::Up), mRight(Vector3Helper::Right)
	{
	}

	SpotLight::~SpotLight()
	{
	}

	const vec3& SpotLight::Direction() const
	{
		return mDirection;
	}
	
	const vec3& SpotLight::Up() const
	{
		return mUp;
	}

	const vec3& SpotLight::Right() const
	{
		return mRight;
	}

	float SpotLight::InnerAngle()
	{
		return mInnerAngle;
	}

	void SpotLight::SetInnerAngle(float value)
	{
		mInnerAngle = value;
	}

	float SpotLight::OuterAngle()
	{
		return mOuterAngle;
	}

	void SpotLight::SetOuterAngle(float value)
	{
		mOuterAngle = value;
	}

    void SpotLight::ApplyRotation(const mat4& transform)
    {
		vec4 direction = transform * vec4(mDirection, 0.0f);
		mDirection = (vec3)normalize(direction);

		vec4 up = transform * vec4(mUp, 0.0f);
		mUp = (vec3)normalize(up);

		mRight = cross(mDirection, mUp);
		mUp = cross(mRight, mDirection);
    }
}