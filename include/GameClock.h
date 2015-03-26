//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		GameClock.h
//\author	William Nguatem
//\note		Copyright (C) 
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26 
//******************************************************************************/


#ifndef _Game_Clock_H_
#define _Game_Clock_H_

#include <exception>

namespace Library
{
	class GameTime;

	typedef union _LARGE_INTEGER1 {
		long long QuadPart;
	} LARGE_INTEGER1;
	
	class GameClock
	{
	public:
		GameClock();

		const LARGE_INTEGER1& StartTime() const;
		const LARGE_INTEGER1& CurrentTime() const;
		const LARGE_INTEGER1& LastTime() const;

		void Reset();
		double GetFrequency() const;
		void GetTime(LARGE_INTEGER1& time) const;
		void UpdateGameTime(GameTime& gameTime);

	private:
		GameClock(const GameClock& rhs);
		GameClock& operator=(const GameClock& rhs);

		LARGE_INTEGER1 mStartTime;
		LARGE_INTEGER1 mCurrentTime;
		LARGE_INTEGER1 mLastTime;
		double mFrequency;
	};
}
#endif//GameClock