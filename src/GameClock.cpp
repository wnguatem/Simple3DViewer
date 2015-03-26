#include <chrono>
#include <iostream>
#include <vector>
#include "GameClock.h"
#include "GameTime.h"

namespace Library
{
	GameClock::GameClock()
		: mStartTime(), mCurrentTime(), mLastTime(), mFrequency()
	{
		mFrequency = GetFrequency();
		Reset();
	}

	const LARGE_INTEGER1& GameClock::StartTime() const
	{
		return mStartTime;
	}

	const LARGE_INTEGER1& GameClock::CurrentTime() const
	{
		return mCurrentTime;
	}

	const LARGE_INTEGER1& GameClock::LastTime() const
	{
		return mLastTime;
	}

	void GameClock::Reset()
	{
		GetTime(mStartTime);
		mCurrentTime = mStartTime;
		mLastTime = mCurrentTime;
	}

	double GameClock::GetFrequency() const
	{
		LARGE_INTEGER1 frequency;
		typedef std::ratio<1l, 1000000000000l> pico;
		typedef std::chrono::duration<long long, pico> picoseconds;
		const auto t1 = std::chrono::high_resolution_clock::now().time_since_epoch().count();

		//if (QueryPerformanceFrequency(&frequency) == false)
		//{
		//	throw std::exception("QueryPerformanceFrequency() failed.");
		//}
		typedef std::chrono::duration<long long> long_duration;
		//if(std::chrono::steady_clock::now() == 0.00)
		//auto t1 = std::chrono::steady_clock::now();
		//(std::chrono::system_clock::now().time_since_epoch()).count();
		//double dd = std::chrono::duration_cast<double>(t1);
		//auto epoch_counter = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

		auto milliseconds_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

		//return (double)t1;
		return (double)milliseconds_since_epoch;
	}

	void GameClock::GetTime(LARGE_INTEGER1& time) const
	{
		//auto epoch_counter = std::chrono::high_resolution_clock::now().time_since_epoch().count();
		//std::chrono::milliseconds(1)
		////QueryPerformanceCounter(&time);
		//time.QuadPart = (long long)epoch_counter;
		auto milliseconds_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

		//return (double)t1;
		time.QuadPart = milliseconds_since_epoch;
	}

	void GameClock::UpdateGameTime(GameTime& gameTime)
	{
		GetTime(mCurrentTime);
		//gameTime.SetTotalGameTime((mCurrentTime.QuadPart - mStartTime.QuadPart) / mFrequency);
		//gameTime.SetElapsedGameTime((mCurrentTime.QuadPart - mLastTime.QuadPart) / mFrequency);
		gameTime.SetTotalGameTime((mCurrentTime.QuadPart - mStartTime.QuadPart)/17.5);
		gameTime.SetElapsedGameTime((mCurrentTime.QuadPart - mLastTime.QuadPart)/17.5);

		mLastTime = mCurrentTime;
	}
}