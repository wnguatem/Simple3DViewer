#include "Utility.h"
#include <algorithm>
#include <exception>
//#include <Shlwapi.h>
// basic file operations
#include <iostream>
#include <fstream>
#include <iterator>
#include <algorithm>
#include <iostream>
#include <boost/filesystem.hpp>


namespace Library
{
	std::string Utility::CurrentDirectory()
	{
		//WCHAR buffer[MAX_PATH];
		//GetCurrentDirectory(MAX_PATH, buffer);
		//std::string currentDirectoryW(buffer);

		//return std::string(currentDirectoryW.begin(), currentDirectoryW.end());
		return " Nichst";
	}

	std::string Utility::ExecutableDirectory()
	{
		//WCHAR buffer[MAX_PATH];
		//GetModuleFileName(nullptr, buffer, MAX_PATH);
		//PathRemoveFileSpec(buffer);

		//return std::string(buffer);
		const char text[] = "olé";
		return (" ");
	}

	void Utility::GetFileName(const std::string& inputPath, std::string& filename)
	{
		std::string fullPath(inputPath);
		std::replace(fullPath.begin(),fullPath.end(),'\\','/');

		std::string::size_type lastSlashIndex = fullPath.find_last_of('/');

		if (lastSlashIndex == std::string::npos)
		{
			filename = fullPath;
		}
		else
		{
			filename = fullPath.substr(lastSlashIndex + 1, fullPath.size() - lastSlashIndex- 1);
		}
	}

	void Utility::GetDirectory(const std::string& inputPath, std::string& directory)
	{
		std::string fullPath(inputPath);
		std::replace(fullPath.begin(),fullPath.end(),'\\','/');

		std::string::size_type lastSlashIndex = fullPath.find_last_of('/');

		if (lastSlashIndex == std::string::npos)
		{
			directory = "";
		}
		else
		{		
			directory = fullPath.substr(0, lastSlashIndex);
		}
	}

	void Utility::GetFileNameAndDirectory(const std::string& inputPath, std::string& directory, std::string& filename)
	{
		std::string fullPath(inputPath);
		std::replace(fullPath.begin(),fullPath.end(),'\\','/');

		std::string::size_type lastSlashIndex = fullPath.find_last_of('/');

		if (lastSlashIndex == std::string::npos)
		{
			directory = "";
			filename = fullPath;
		}
		else
		{
			directory = fullPath.substr(0, lastSlashIndex);
			filename = fullPath.substr(lastSlashIndex + 1, fullPath.size() - lastSlashIndex- 1);
		}
	}
	
	void Utility::LoadBinaryFile(const std::string& filename, std::vector<char>& data)
	{
        //setup converter
        std::string converted_str;
        for(char x : filename)
            converted_str += x;
     
 //       /use converter (.to_bytes: wstr->str, .from_bytes: str->wstr)
        //std::string converted_str = converter.to_bytes( filename);
        //std::string filename_1 = filename;
        
        std::ifstream file_a(converted_str.c_str(), std::ios::binary);
		if (file_a.bad())
		{
			//throw std::exception("Could not open file.");
		}
        
		if (!file_a.good())
		{
			std::cout << "Can't find my file: " << converted_str << std::endl;
			//throw std::exception("Could not open file.");
		}

		if (!boost::filesystem::exists(converted_str.c_str()))
		{
			std::cout << "Can't find my file: "<<converted_str<< std::endl;
		}        
        
		file_a.seekg(0, std::ios::end);
		unsigned int size = (unsigned int)file_a.tellg();

		if (size > 0)
		{
			data.resize(size);
			file_a.seekg(0, std::ios::beg);
			file_a.read(&data.front(), size);
		}

		file_a.close();
	}

	void Utility::ToWideString(const std::string& source, std::string& dest)
	{
		dest.assign(source.begin(), source.end());
	}

	std::string Utility::ToWideString(const std::string& source)
	{
		std::string dest;
		dest.assign(source.begin(), source.end());

		return dest;
	}

	void Utility::ToString(const std::string& source, std::string& dest)
	{
		dest.assign(source.begin(), source.end());
	}

	std::string Utility::ToString(const std::string& source)
	{
		std::string dest;
		dest.assign(source.begin(), source.end());

		return dest;
	}

	void Utility::PathJoin(std::string& dest, const std::string& sourceDirectory, const std::string& sourceFile)
	{
		//WCHAR buffer[MAX_PATH];

		//PathCombine(buffer, sourceDirectory.c_str(), sourceFile.c_str());
		//dest = buffer;
	}

	void Utility::GetPathExtension(const std::string& source, std::string& dest)
	{
		//dest = PathFindExtension(source.c_str());
	}
}