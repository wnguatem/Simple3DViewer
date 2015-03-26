//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		ServiceContainer.h
//\author	William Nguatem
//\note		Copyright (C) 
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26 
//******************************************************************************/


#ifndef _Service_Container_H_
#define _ServiceContainer_H_
#include <map>

namespace Library
{
	class ServiceContainer
	{
	public:
		ServiceContainer();

		void AddService(unsigned int typeID, void* service);
		void RemoveService(unsigned int typeID);
		void* GetService(unsigned int typeID) const;

	private:
		ServiceContainer(const ServiceContainer& rhs);
		ServiceContainer& operator=(const ServiceContainer& rhs);

		std::map<unsigned int, void*> mServices;
	};
}
#endif//_Service_Container_H_
