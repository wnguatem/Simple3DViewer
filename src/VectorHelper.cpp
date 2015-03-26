#include <sstream>
#include "VectorHelper.h"

namespace Library
{
	const glm::vec2 Vector2Helper::Zero = glm::vec2(0.0f, 0.0f);
	const glm::vec2 Vector2Helper::One = glm::vec2(1.0f, 1.0f);

	std::string Vector2Helper::ToString(const glm::vec2& vector)
	{
		std::stringstream stream;

		stream << "{" << vector.x << ", " << vector.y << "}";

		return stream.str();
	}

	const glm::vec3 Vector3Helper::Zero = glm::vec3(0.0f, 0.0f, 0.0f);
	const glm::vec3 Vector3Helper::One = glm::vec3(1.0f, 1.0f, 1.0f);
	const glm::vec3 Vector3Helper::Forward = glm::vec3(0.0f, 0.0f, -1.0f);
	const glm::vec3 Vector3Helper::Backward = glm::vec3(0.0f, 0.0f, 1.0f);
	const glm::vec3 Vector3Helper::Up = glm::vec3(0.0f, 1.0f, 0.0f);
	const glm::vec3 Vector3Helper::Down = glm::vec3(0.0f, -1.0f, 0.0f);
	const glm::vec3 Vector3Helper::Right = glm::vec3(1.0f, 0.0f, 0.0f);
	const glm::vec3 Vector3Helper::Left = glm::vec3(-1.0f, 0.0f, 0.0f);

	std::string Vector3Helper::ToString(const glm::vec3& vector)
	{
		std::stringstream stream;

		stream << "{" << vector.x << ", " << vector.y << ", " << vector.z << "}";

		return stream.str();
	}

	const glm::vec4 Vector4Helper::Zero = glm::vec4(0.0f, 0.0f, 0.0f, 0.0f);
	const glm::vec4 Vector4Helper::One = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);

	std::string Vector4Helper::ToString(const glm::vec4& vector)
	{
		std::stringstream stream;

		stream << "{" << vector.x << ", " << vector.y << ", " << vector.z << ", " << vector.w << "}";

		return stream.str();
	}
}
