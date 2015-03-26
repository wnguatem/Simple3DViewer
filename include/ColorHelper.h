//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		ColorHelper.h
//\author	William Nguatem
//\note		Copyright (C) 
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26 
//******************************************************************************/


#ifndef _ColorHelper_H_
#define _ColorHelper_H_

#include "Common.h"
#include <random>

namespace Library
{
	class ColorHelper
	{
	public:
		static const glm::vec4 Black;
		static const glm::vec4 White;
		static const glm::vec4 Red;
		static const glm::vec4 Green;
		static const glm::vec4 Blue;
		static const glm::vec4 Yellow;
		static const glm::vec4 BlueGreen;
		static const glm::vec4 Purple;
		static const glm::vec4 CornflowerBlue;
		static const glm::vec4 Wheat;
		static const glm::vec4 LightGray;

		static glm::vec4 RandomColor();

	private:
		static std::random_device sDevice;
		static std::default_random_engine sGenerator;
		static std::uniform_real_distribution<float> sDistribution;

		ColorHelper();
		ColorHelper(const ColorHelper& rhs);
		ColorHelper& operator=(const ColorHelper& rhs);
	};
}
#endif//_ColorHelper_H_