#include "ColorHelper.h"

namespace Library
{
	std::random_device ColorHelper::sDevice;
	std::default_random_engine ColorHelper::sGenerator(sDevice());
	std::uniform_real_distribution<float> ColorHelper::sDistribution(0, 1);

	const glm::vec4 ColorHelper::Black =			glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
	const glm::vec4 ColorHelper::White =			glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);
	const glm::vec4 ColorHelper::Red =				glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
	const glm::vec4 ColorHelper::Green =			glm::vec4(0.0f, 1.0f, 0.0f, 1.0f);
	const glm::vec4 ColorHelper::Blue =				glm::vec4(0.0f, 0.0f, 1.0f, 1.0f);
	const glm::vec4 ColorHelper::Yellow =			glm::vec4(1.0f, 1.0f, 0.0f, 1.0f);
	const glm::vec4 ColorHelper::BlueGreen =		glm::vec4(0.0f, 1.0f, 1.0f, 1.0f);
	const glm::vec4 ColorHelper::Purple =			glm::vec4(1.0f, 0.0f, 1.0f, 1.0f);
	const glm::vec4 ColorHelper::CornflowerBlue =	glm::vec4(0.392f, 0.584f, 0.929f, 1.0f);
	const glm::vec4 ColorHelper::Wheat =			glm::vec4(0.961f, 0.871f, 0.702f, 1.0f);
	const glm::vec4 ColorHelper::LightGray =		glm::vec4(0.5f, 0.5f, 0.5f, 1.0f);

	glm::vec4 ColorHelper::RandomColor()
	{
		float r = sDistribution(sGenerator);
		float g = sDistribution(sGenerator);
		float b = sDistribution(sGenerator);

		return glm::vec4(r, g, b, 1.0f);
	}
}
