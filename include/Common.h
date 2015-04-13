//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		Common.h
//\author	William Nguatem
//\note		Copyright (C) 
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26 
//******************************************************************************/


#ifndef _Common_H_
#define _Common_H_

#include <exception>
#include <cassert>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include "ServiceContainer.h"

//#if defined(OPENGL)
#include "GL/gl3w.h"
#include "GLFW/glfw3.h"
#include "glm/glm.hpp"
#include "glm/gtx/simd_mat4.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/matrix_inverse.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/detail/func_trigonometric.hpp"

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))
/* UNIX-style OS. ------------------------------------------- */
#if defined(__APPLE__) && defined(__MACH__)
/* Apple OSX and iOS (Darwin) */
#define GLFW_EXPOSE_NATIVE_COCOA
#define GLFW_EXPOSE_NATIVE_NSGL
#include <TargetConditionals.h>
#if TARGET_IPHONE_SIMULATOR == 1
/* iOS in Xcode simulator */
#elif TARGET_OS_IPHONE == 1
/* iOS on iPhone, iPad, etc. */
#elif TARGET_OS_MAC == 1
/* OS X */
#endif
#endif
#endif

#if defined(_WIN64)

/* Microsoft Windows (64-bit) */
#define GLFW_EXPOSE_NATIVE_WIN32
#define GLFW_EXPOSE_NATIVE_WGL

#elif defined(_WIN32)
/* Microsoft Windows (32-bit) */
#define GLFW_EXPOSE_NATIVE_WIN32
#define GLFW_EXPOSE_NATIVE_WGL

#endif


//*  The available window API macros are:
//*  * `GLFW_EXPOSE_NATIVE_WIN32`
//*  * `GLFW_EXPOSE_NATIVE_COCOA`
//*  * `GLFW_EXPOSE_NATIVE_X11`
//*
//*  The available context API macros are:
//*  * `GLFW_EXPOSE_NATIVE_WGL`
//*  * `GLFW_EXPOSE_NATIVE_NSGL`
//*  * `GLFW_EXPOSE_NATIVE_GLX`
//*  * `GLFW_EXPOSE_NATIVE_EGL`


//#endif

typedef struct 
{ 
	long x;
	long y;
} POINT1;


#define DeleteObject(object) if((object) != NULL) { delete object; object = NULL; }
#define DeleteObjects(objects) if((objects) != NULL) { delete[] objects; objects = NULL; }

namespace Library
{
	typedef unsigned char byte;

	extern ServiceContainer GlobalServices;
}
#endif//_Common_H_