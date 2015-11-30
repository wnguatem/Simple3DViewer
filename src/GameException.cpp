#include "GameException.h"

namespace Library
{
	GameException::GameException(const char* const& message)
		//: exception(message)
	{
	}

	std::string GameException::whatw() const
	{
		std::string whatString(what());
		std::string whatw;
		whatw.assign(whatString.begin(), whatString.end());

		return whatw;
	}
}