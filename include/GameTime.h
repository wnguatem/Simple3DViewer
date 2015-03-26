//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		GameTime.h
//\author	William Nguatem
//\note		Copyright (C) 
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26 
//******************************************************************************/


#ifndef _Game_Time_H_
#define _Game_Time_H_

namespace Library
{
	class GameTime
	{
	public:
		GameTime();
		GameTime(double totalGameTime, double elapsedGameTime);

		double TotalGameTime() const;
		void SetTotalGameTime(double totalGameTime);

		double ElapsedGameTime() const;
		void SetElapsedGameTime(double elapsedGameTime);

	private:
		double mTotalGameTime;
		double mElapsedGameTime;
	};
}
#endif//_Game_Time_H_
