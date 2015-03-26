#include "Light.h"
#include "ColorHelper.h"

namespace Library
{
	//RTTI_DEFINITIONS(Light)

	Light::Light(Game& game)
		: GameComponent(game), mColor(ColorHelper::White)
	{
	}

	Light::~Light()
	{
	}

	const glm::vec4& Light::Color() const
	{
		return mColor;
	}

	void Light::SetColor(float r, float g, float b, float a)
	{
		glm::vec4 color(r, g, b, a);
		SetColor(color);
	}

	void Light::SetColor(const glm::vec4& color)
	{
		mColor = color;
	}
}