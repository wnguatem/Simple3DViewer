//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		Variable.h
//\author	William Nguatem
//\note		Copyright (C) 
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26 
//******************************************************************************/


#ifndef _Variable_H_
#define _Variable_H_

#include "Common.h"

namespace Library
{
	class ShaderProgram;

    class Variable
    {
    public:
		Variable(ShaderProgram& shaderProgram, const std::string& name);		

        ShaderProgram& GetShaderProgram();
		const GLint& Location() const;
        const std::string& Name() const;

        Variable& operator<<(const glm::mat4& value);
        Variable& operator<<(const glm::vec4& value);
		Variable& operator<<(const glm::vec3& value);
		Variable& operator<<(const glm::vec2& value);        
		Variable& operator<<(float value);
		Variable& operator<<(int value);

    private:
        Variable(const Variable& rhs);
        Variable& operator=(const Variable& rhs);

		ShaderProgram& mShaderProgram;
		GLint mLocation;
        std::string mName;
    };
}
#endif//_Variable_H_