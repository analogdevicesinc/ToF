// Copyright (c) Microsoft Corporation. All rights reserved.
// Modified by Analog Devices, Inc.
// Licensed under the MIT License.

#ifndef ADISHADER_H
#define ADISHADER_H

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <GL/gl3w.h>
#include <list>

namespace adiviewer
{
	
	class ADIShader
	{
	public:

		/**
		* @brief Constructor
		*        Generates a shader type and compiles it
		*/
		ADIShader(GLenum shaderType, const GLchar* source)
		{
			ID = glCreateShader(shaderType);
			glShaderSource(ID, 1, &source, nullptr);
			glCompileShader(ID);

			GLint success = GL_FALSE;
			char infoLog[512];
			glGetShaderiv(ID, GL_COMPILE_STATUS, &success);
			if (!success)
			{
				glGetShaderInfoLog(ID, 512, nullptr, infoLog);
				std::stringstream errorBuilder;
				errorBuilder << "Shader compilation error: " << std::endl << infoLog;
				throw std::logic_error(errorBuilder.str().c_str());
			}
		}

		/**
		* @brief Activate Shader Program
		*/
		ADIShader(ADIShader&& other) :ID(other.ID)
		{
			other.ID = 0;
		}

		ADIShader& operator=(ADIShader&& other)
		{
			if (this != &other)
			{
				ID = other.ID;
				other.ID = 0;
			}
			return *this;
		}

		GLuint Id()
		{
			return ID;
		}

		~ADIShader()
		{
			glDeleteShader(ID);
		}

		ADIShader(const ADIShader&) = delete;
		ADIShader& operator =(const ADIShader&) = delete;

		/**
		* @brief Activate Shader Program
		*/
		void useProgram() const
		{
			glUseProgram(ID);
		}

	private:

		GLuint ID = 0;

	};

	class Program
	{
		public:
			Program()
			{
				
			}

			void AttachShader(ADIShader&& newShader)
			{
				glAttachShader(ID, newShader.Id());
				shaders.emplace_back(std::move(newShader));
			}

			void RemoveShaders()
			{
				shaders.clear();
			}

			void CreateProgram()
			{
				ID = glCreateProgram();
			}

			void Link()
			{
				glLinkProgram(ID);
				GLint success = GL_FALSE;
				char infoLog[512];
				std::string read = "";
				glGetProgramiv(ID, GL_LINK_STATUS, &success);
				if (!success)
				{
					glGetProgramInfoLog(ID, 512, nullptr, infoLog);
					std::stringstream errorBuilder;
					errorBuilder << "Shader program linking error: " << std::endl << infoLog;
					read = infoLog;
					throw std::logic_error(errorBuilder.str().c_str());
				}
			}

			GLint GetUniformLocation(const GLchar* name)
			{
				return glGetUniformLocation(ID, name);
			}

			GLuint Id()
			{
				return ID;
			}

			~Program()
			{
				// Reset the active shader if we're about to delete it
				//
                if (ID != 0) {
                    GLint currentProgramId;
                    glGetIntegerv(GL_CURRENT_PROGRAM, &currentProgramId);
                    if (ID == static_cast<GLuint>(currentProgramId))
                    {
                        glUseProgram(0);
                    }

                    glDeleteProgram(ID);
                }
			}

			Program(const Program&) = delete;
			Program& operator=(const Program&) = delete;

		private:
			GLuint ID = 0;
			std::list<ADIShader> shaders;
	};
	
}//adiviewer

#endif
