//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		MatrixHelper.h
//\author	William Nguatem
//\note		Copyright (C) 
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26 
//******************************************************************************/


#ifndef _Matrix_Helper_H_
#define _Matrix_Helper_H_

#include "Common.h"

namespace Library
{
	class MatrixHelper
	{
	public:
		static void GetForward(glm::mat4& matrix, glm::vec3& vector);
		static void GetUp(glm::mat4& matrix, glm::vec3& vector);
		static void GetRight(glm::mat4& matrix, glm::vec3& vector);
		static void GetTranslation(glm::mat4& matrix, glm::vec3& vector);

		static void SetForward(glm::mat4& matrix, glm::vec3& forward);
		static void SetUp(glm::mat4& matrix, glm::vec3& up);
		static void SetRight(glm::mat4& matrix, glm::vec3& right);
		static void SetTranslation(glm::mat4& matrix, glm::vec3& translation);

	private:
		MatrixHelper();
		MatrixHelper(const MatrixHelper& rhs);
		MatrixHelper& operator=(const MatrixHelper& rhs);
	};
}
#endif//_Matrix_Helper_H_