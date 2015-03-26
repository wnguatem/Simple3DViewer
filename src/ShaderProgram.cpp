#include "ShaderProgram.h"
#include "GameException.h"
#include "Utility.h"
#include "Model.h"
#include "Mesh.h"
#include <sstream>

namespace Library
{
	//RTTI_DEFINITIONS(ShaderProgram)

	ShaderProgram::ShaderProgram()
		: mProgram(0), mVariables(), mVariablesByName()
	{
		mProgram = glCreateProgram();
	}

	ShaderProgram::~ShaderProgram()
	{
		glDeleteProgram(mProgram);

		for (Variable* variable : mVariables)
		{
			delete variable;
		}
	}

	GLuint ShaderProgram::CompileShaderFromFile(GLenum shaderType, const std::wstring& filename)
	{
		std::vector<char> shaderSource;
		Utility::LoadBinaryFile(filename, shaderSource);
		GLchar* sourcePointer = &shaderSource[0];
		GLint length = shaderSource.size();

		GLuint shader = glCreateShader(shaderType);
		glShaderSource(shader, 1, &sourcePointer, &length);
		glCompileShader(shader);

		GLint compileStatus;
		glGetShaderiv(shader, GL_COMPILE_STATUS, &compileStatus);
		if (compileStatus != GL_TRUE)
		{
			GLint logLength;
			glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLength);

			std::string log;
			log.reserve(logLength);

			glGetShaderInfoLog(shader, logLength, nullptr, const_cast<GLchar*>(log.c_str()));

			std::stringstream errorMessage;
			errorMessage << "glCompileShader() failed.\n" << log.c_str();

			throw GameException(errorMessage.str().c_str());
		}

		return shader;
	}

	GLuint ShaderProgram::Program() const
	{
		return mProgram;
	}

	const std::vector<Variable*>& ShaderProgram::Variables() const
	{
		return mVariables;
	}

	const std::map<std::string, Variable*>& ShaderProgram::VariablesByName() const
	{
		return mVariablesByName;
	}

	void ShaderProgram::BuildProgram(const std::vector<ShaderDefinition>& shaderDefinitions)
	{
		std::vector<GLuint> compiledShaders;
		compiledShaders.reserve(shaderDefinitions.size());

		for (ShaderDefinition shaderDefiniton : shaderDefinitions)
		{
			GLuint compiledShader = CompileShaderFromFile(shaderDefiniton.first, shaderDefiniton.second);
			glAttachShader(mProgram, compiledShader);
			compiledShaders.push_back(compiledShader);
		}

		glLinkProgram(mProgram);
		GLint linkStatus;
		glGetProgramiv(mProgram, GL_LINK_STATUS, &linkStatus);
		if (linkStatus != GL_TRUE)
		{
			GLint logLength;
			glGetProgramiv(mProgram, GL_INFO_LOG_LENGTH, &logLength);

			std::string log;
			log.reserve(logLength);

			glGetProgramInfoLog(mProgram, logLength, nullptr, const_cast<GLchar*>(log.c_str()));

			std::stringstream errorMessage;
			errorMessage << "glCompileShader() failed.\n" << log.c_str();

			throw GameException(errorMessage.str().c_str());
		}

		for (GLuint compiledShader : compiledShaders)
		{
			glDeleteShader(compiledShader);
		}
	}

	void ShaderProgram::Use() const
	{
		glUseProgram(mProgram);
	}

	void ShaderProgram::CreateVertexBuffer(const Model& model, std::vector<GLuint>& vertexBuffers) const
	{
		vertexBuffers.reserve(model.Meshes().size());
		for (Mesh* mesh : model.Meshes())
		{
			GLuint vertexBuffer;
			CreateVertexBuffer(*mesh, vertexBuffer);
			vertexBuffers.push_back(vertexBuffer);
		}
	}

	void ShaderProgram::Initialize(GLuint vertexArrayObject)
	{
		glBindVertexArray(vertexArrayObject);
	}

	void ShaderProgram::CreateVertexBuffer(const Mesh& mesh, GLuint& vertexBuffer) const
	{
		throw GameException("ShaderProgram::CreateVertexBuffer() not implemented for base class.");
	}

	unsigned int ShaderProgram::VertexSize() const
	{
		throw GameException("ShaderProgram::VertexSize() not implemented for base class.");
	}
}