//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		Utility.h
//\author	William Nguatem
//\note		Copyright (C) 
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26 
//******************************************************************************/


#ifndef _Utility_H_
#define _Utility_H_

#include "Common.h"

namespace Library
{
	class Utility
	{
	public:
		static std::string CurrentDirectory();
		static std::string ExecutableDirectory();
		static void GetFileName(const std::string& inputPath, std::string& filename);
		static void GetDirectory(const std::string& inputPath, std::string& directory);
		static void GetFileNameAndDirectory(const std::string& inputPath, std::string& directory, std::string& filename);
		static void LoadBinaryFile(const std::string& filename, std::vector<char>& data);
		static void ToWideString(const std::string& source, std::string& dest);
		static std::string ToWideString(const std::string& source);
		static void ToString(const std::string& source, std::string& dest);
		static std::string ToString(const std::string& source);
		static void PathJoin(std::string& dest, const std::string& sourceDirectory, const std::string& sourceFile);
		static void GetPathExtension(const std::string& source, std::string& dest);

	private:
		Utility();
		Utility(const Utility& rhs);
		Utility& operator=(const Utility& rhs);
	};
}
#endif//_Utility_H_s