#include "GameException.h"

namespace Library
{
	GameException::GameException(const char* const& message)
		//: exception(message)
	{
	}

	std::wstring GameException::whatw() const
	{
		std::string whatString(what());
		std::wstring whatw;
		whatw.assign(whatString.begin(), whatString.end());

		return whatw;
	}
}