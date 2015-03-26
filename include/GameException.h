//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		GameException.h
//\author	William Nguatem
//\note		Copyright (C) 
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26 
//******************************************************************************/


#ifndef _Game_Exception_H_
#define _Game_Exception_H_

#include <exception>
//#include <Windows.h>
#include <string>

namespace Library
{
	class GameException : public std::exception
	{
	public:
		GameException(const char* const& message);

		//HRESULT HR() const;
		std::wstring whatw() const;

	//private:
	//	HRESULT mHR;
	};
}
#endif//_Game_Exception_H_