PROJECT(simplecool3dviewer)


# Try to Find OpenGL and GLUT silently
# In addition sets two flags if the found versions are Apple frameworks
# OPENGL_IS_A_FRAMEWORK
# GLUT_IS_A_FRAMEWORK

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

if(APPLE AND OPENGL_FOUND)
  if ("${OPENGL_INCLUDE_DIR}" MATCHES "\\.framework")
    set(OPENGL_IS_A_FRAMEWORK TRUE)
  endif ("${OPENGL_INCLUDE_DIR}" MATCHES "\\.framework")
endif(APPLE AND OPENGL_FOUND)

find_package(GLUT QUIET)

if(APPLE AND GLUT_FOUND)
  if ("${GLUT_INCLUDE_DIR}" MATCHES "\\.framework")
    set(GLUT_IS_A_FRAMEWORK TRUE)
  endif ("${GLUT_INCLUDE_DIR}" MATCHES "\\.framework")
endif(APPLE AND GLUT_FOUND)

 IF(APPLE)
    INCLUDE_DIRECTORIES ( /System/Library/Frameworks )
    FIND_LIBRARY(COCOA_LIBRARY Cocoa)
    FIND_LIBRARY(GLUT_LIBRARY GLUT )
    FIND_LIBRARY(OpenGL_LIBRARY OpenGL )
    MARK_AS_ADVANCED (COCOA_LIBRARY
                      GLUT_LIBRARY
                      OpenGL_LIBRARY)
    SET(EXTRA_LIBS ${COCOA_LIBRARY} ${GLUT_LIBRARY} ${OpenGL_LIBRARY})
 ENDIF (APPLE)
 #target_link_libraries(Test ${EXTRA_LIBS})
 IF(APPLE)
set (LIB_FOLDER /Users/williamnguatem/projects/libs)
 set(KML_INCLUDE /Users/williamnguatem/projects/libs/libkml/src)
set(XERCES_INCLUDE /Users/williamnguatem/projects/libs/xerces-c-3.1.2/src)
include_directories(/Users/williamnguatem/projects/libs/gl3w/include)
include_directories(/Users/williamnguatem/projects/libs/glfw/include)
include_directories(/Users/williamnguatem/projects/libs/glm)
include_directories(/Users/williamnguatem/projects/libs/assimp/include)
#include_directories(/Users/williamnguatem/projects/libs/libSOIL)
include_directories(/Users/williamnguatem/projects/libs/Simple-OpenGL-Image-Library/src)
 set(KML_INCLUDE /Users/williamnguatem/projects/libs/libkml/src)

 set(ASSIMP_LIB /Users/williamnguatem/projects/libs/assimp/build/code/Debug/libassimpd.dylib)
 set(GLFW_LIB /Users/williamnguatem/projects/libs/glfw/build/src/Debug/libglfw.3.1.dylib)
#set(SOIL_LIB /Users/williamnguatem/projects/libs/libSOIL/libSOIL.dylib)
set(SOIL_LIB /Users/williamnguatem/projects/libs/Simple-OpenGL-Image-Library/build/Debug/libSOIL.a)
 set(GL3W /Users/williamnguatem/projects/libs/gl3w/build/Debug/libgl3w.a)
ENDIF (APPLE)

 IF(WIN32)
 include_directories(C:/williamnguatem/Projects/LIBS/vs2013/gl3w/include)
 include_directories(C:/williamnguatem/Projects/LIBS/vs2013/glfw/include)
 include_directories(C:/williamnguatem/Projects/LIBS/vs2013/glm)
 include_directories(C:/williamnguatem/Projects/LIBS/vs2013/assimp/include)
 include_directories(C:/williamnguatem/Projects/LIBS/vs2013/soil/SOIL/src) 
  
 set(ASSIMP_LIB C:/williamnguatem/Projects/LIBS/vs2013/assimp/build/code/Debug/assimp-vc120-mtd.lib)
 set(GL3W C:/williamnguatem/Projects/LIBS/vs2013/gl3w/build/Debug/gl3w.lib)
 set(GLFW_LIB C:/williamnguatem/Projects/LIBS/vs2013/glfw/build/src/Debug/glfw3.lib)
 set(SOIL_LIB C:/williamnguatem/Projects/LIBS/vs2013/soil/SOIL/lib/SOIL_d.lib)
 set(ESRI_SHAPEFILE_LIB C:/williamnguatem/Projects/ReadEsriShapeFiles/shp-src-02/contrib/msvc/lib/shapelibd.lib)
 set(XERCES_LIB C:/williamnguatem/Projects/LIBS/vs2013/xerces-c-3.1.2/Build/Win64/VC12/Debug/xerces-c_3D.lib)
 set(XERCES_INCLUDE C:/williamnguatem/Projects/LIBS/vs2013/xerces-c-3.1.2/src)
 set(KML_INCLUDE C:/williamnguatem/Projects/LIBS/vs2013/libkml/src)
 set(KML_LIB C:/williamnguatem/Projects/LIBS/vs2013/libkml/msvc/x64/Debug/libkmlbase.lib
			C:/williamnguatem/Projects/LIBS/vs2013/libkml/msvc/x64/Debug/libkmlxsd.lib
			C:/williamnguatem/Projects/LIBS/vs2013/libkml/msvc/x64/Debug/libkmlregionator.lib
			C:/williamnguatem/Projects/LIBS/vs2013/libkml/msvc/x64/Debug/libkmlengine.lib
			C:/williamnguatem/Projects/LIBS/vs2013/libkml/msvc/x64/Debug/libkmldom.lib
			C:/williamnguatem/Projects/LIBS/vs2013/libkml/msvc/x64/Debug/libkmlbase.lib
			C:/williamnguatem/Projects/LIBS/vs2013/libkml/msvc/x64/Debug/libkmlconvenience.lib)
 ENDIF (WIN32)

 
find_package(Qt5Widgets)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
 

#set modulname or set sub_system_name ${FI_INCLUDE_DIR}
SET(simplecool3dviewer_HEADERS
    include/Common.h
    include/Game.h
    include/GameClock.h
    include/GameComponent.h
    include/GameTime.h
    include/ServiceContainer.h
    include/DrawableGameComponent.h
    include/Factory.h
    include/Utility.h
    include/Camera.h
    include/ColorHelper.h
    include/ShaderProgram.h
    include/VertexDeclarations.h      
    include/VectorHelper.h
    include/FirstPersonCamera.h
    include/Grid.h
    include/Mesh.h
    include/Model.h
    include/ModelMaterial.h
    include/Light.h
    include/PointLight.h
    include/SpotLight.h
    include/DirectionalLight.h
    include/ProxyModel.h
    include/MatrixHelper.h
    include/BasicEffect.h
    include/Skybox.h
    include/SkyboxEffect.h
    include/Variable.h
    include/GameException.h
    include/QuadModeler.h
	include/EsriShapeFileModeler.h
	include/KMLModeler.h
	include/CityGMLModeler.h
	include/shapefil.h
    include/TexturedQuad.h
    include/Quad.h
    include/Quads.h
    include/SpotLightEffect.h
    include/SpotLightDemo.h
    include/NormalMappingDemo.h
    include/NormalMappingEffect.h
    include/FogEffect.h
    include/TransparencyMappingDemo.h
    include/FogDemo.h
    include/TransparencyMappingEffect.h
    include/EnvironmentMappingDemo.h
    include/EnvironmentMappingEffect.h
    include/LineStrip.h
	RenderingGame.h
	TexturedModelDemo.h
VisPointCloud.h
)
  
SET(simplecool3dviewer_SOURCES
    src/Quad.cpp
    src/Quads.cpp
	src/Game.cpp
    src/TexturedQuad.cpp
    src/LineStrip.cpp
   src/GameClock.cpp
   src/GameComponent.cpp
   src/GameTime.cpp
   src/ServiceContainer.cpp
   src/DrawableGameComponent.cpp
   src/Utility.cpp
   src/Camera.cpp
   src/Common.cpp
   src/ColorHelper.cpp
   src/ShaderProgram.cpp
   src/VectorHelper.cpp
   src/FirstPersonCamera.cpp
   src/Grid.cpp
   src/Mesh.cpp
   src/Model.cpp
   src/ModelMaterial.cpp
   src/DirectionalLight.cpp
   src/Light.cpp
   src/PointLight.cpp
   src/SpotLight.cpp
   src/ProxyModel.cpp
   src/MatrixHelper.cpp
   src/Variable.cpp
   src/BasicEffect.cpp
   src/Skybox.cpp
   src/SkyboxEffect.cpp
   src/GameException.cpp
   src/QuadModeler.cpp
   src/EsriShapeFileModeler.cpp
   src/KMLModeler.cpp
   src/CityGMLModeler.cpp
   src/SpotLightEffect.cpp
   src/SpotLightDemo.cpp
   src/NormalMappingDemo.cpp
   src/FogEffect.cpp
   src/NormalMappingEffect.cpp
   src/TransparencyMappingDemo.cpp
   src/TransparencyMappingEffect.cpp
   src/EnvironmentMappingDemo.cpp
   src/EnvironmentMappingEffect.cpp
   src/FogDemo.cpp
	TexturedModelDemo.cpp
	RenderingGame.cpp
	Program.cpp
VisPointCloud.cpp
 )
#   src/gl3w.c

#INCLUDE_DIRECTORIES(/Users/williamnguatem/projects/new_vis_tool/cmakeproject)
#INCLUDE_DIRECTORIES(/Users/williamnguatem/projects/new_vis_tool/cmakeproject/include)

 INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include)
 INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
 include_directories(${OPENGL_INCLUDE_DIRS})

 include_directories(${XERCES_INCLUDE})
 include_directories(${KML_INCLUDE})


#open gl libraries includes glu . . .
#set( GLFW_BUILD_EXAMPLES OFF CACHE BOOL "turn off build examples")
#add_subdirectory(glfw)
#link_directories(${PROJECT_BINARY_DIR})
#link_directories(${GLFW_BINARY_DIR})

if(WINDOWS)
link_directories(${PROJECT_SOURCE_DIR}/win/glew/lib/)
endif(WINDOWS)

##FIND_PACKAGE( OpenCV REQUIRED )
##SET(BUILD_SHARED_LIBS ON)
##IF(NOT BUILD_SHARED_LIBS)
##  ADD_DEFINITIONS( -DANN_STATIC_LIB )
##ENDIF()

#include_directories(${PROJECT_SOURCE_DIR}/include/)
#include_directories(${PROJECT_SOURCE_DIR})
#include_directories(${PROJECT_SOURCE_DIR}/glfw/include/)


#Force PCL to be used
IF(APPLE)
set(PCL_LIBS_DIR /Users/williamnguatem/projects/libs/pcl/build/lib/Debug)
include_directories(/Users/williamnguatem/projects/libs/pcl/io/include)
include_directories(/Users/williamnguatem/projects/libs/pcl/2d/include)
include_directories(/Users/williamnguatem/projects/libs/pcl/common/include)
include_directories(/Users/williamnguatem/projects/libs/pcl/features/include)
include_directories(/Users/williamnguatem/projects/libs/pcl/filters/include)
include_directories(/Users/williamnguatem/projects/libs/pcl/geometry/include)
include_directories(/Users/williamnguatem/projects/libs/pcl/kdtree/include)
include_directories(/Users/williamnguatem/projects/libs/pcl/ml/include)
include_directories(/Users/williamnguatem/projects/libs/pcl/octree/include)
include_directories(/Users/williamnguatem/projects/libs/pcl/recognition/include)
include_directories(/Users/williamnguatem/projects/libs/pcl/registration/include)
include_directories(/Users/williamnguatem/projects/libs/pcl/sample_consensus/include)
include_directories(/Users/williamnguatem/projects/libs/pcl/search/include)
include_directories(/Users/williamnguatem/projects/libs/pcl/segmentation/include)
include_directories(/Users/williamnguatem/projects/libs/pcl/stereo/include)
include_directories(/Users/williamnguatem/projects/libs/pcl/surface/include)
include_directories(/Users/williamnguatem/projects/libs/pcl/tracking/include)
include_directories(/opt/local/include)

set( PCLReleaselibs
${Boost_LIBRARIES}
${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE}/Release/io_Release
${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE}/model_segmentation_Release
${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE}/vp_detection_Release
${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE}/linear_algebra_Release
${IVT_LIB_DIR}/lib/win32/ivt.lib
)
set( PCLDebuglibs
${PCL_LIBS_DIR}/libpcl_search.dylib
${PCL_LIBS_DIR}/libpcl_kdtree.dylib
${PCL_LIBS_DIR}/libpcl_segmentation.dylib
${PCL_LIBS_DIR}/libpcl_io.dylib
${PCL_LIBS_DIR}/libpcl_tracking.dylib
${PCL_LIBS_DIR}/libpcl_sample_consensus.dylib
${PCL_LIBS_DIR}/libpcl_ml.dylib
${PCL_LIBS_DIR}/libpcl_octree.dylib
${PCL_LIBS_DIR}/libpcl_stereo.dylib
${PCL_LIBS_DIR}/libpcl_surface.dylib
${PCL_LIBS_DIR}/libpcl_registration.dylib
${PCL_LIBS_DIR}/libpcl_recognition.dylib
${PCL_LIBS_DIR}/libpcl_people.dylib
${PCL_LIBS_DIR}/libpcl_features.dylib
${PCL_LIBS_DIR}/libpcl_keypoints.dylib
${PCL_LIBS_DIR}/libpcl_common.dylib
${PCL_LIBS_DIR}/libpcl_filters.dylib
${PCL_LIBS_DIR}/libpcl_io_ply.dylib
${PCL_LIBS_DIR}/libpcl_2d.dylib
 /Users/williamnguatem/projects/libs/shapelib-1.3.0/libshp.a
)

#set( PCLDebuglibs
#debug ${PCL_LIBS_DIR}/libpcl_search.dylib
#debug ${PCL_LIBS_DIR}/libpcl_kdtree.dylib
#debug ${PCL_LIBS_DIR}/libpcl_segmentation.dylib
#debug ${PCL_LIBS_DIR}/libpcl_io.dylib
#debug ${PCL_LIBS_DIR}/libpcl_tracking.dylib
#debug ${PCL_LIBS_DIR}/libpcl_sample_consensus.dylib
#debug ${PCL_LIBS_DIR}/libpcl_ml.dylib
#debug ${PCL_LIBS_DIR}/libpcl_octree.dylib
#debug ${PCL_LIBS_DIR}/libpcl_stereo.dylib
#debug ${PCL_LIBS_DIR}/libpcl_surface.dylib
#debug ${PCL_LIBS_DIR}/libpcl_registration.dylib
#debug ${PCL_LIBS_DIR}/libpcl_recognition.dylib
#debug ${PCL_LIBS_DIR}/libpcl_people.dylib
#debug ${PCL_LIBS_DIR}/libpcl_features.dylib
#debug ${PCL_LIBS_DIR}/libpcl_keypoints.dylib
#debug ${PCL_LIBS_DIR}/libpcl_common.dylib
#debug ${PCL_LIBS_DIR}/libpcl_filters.dylib
#debug ${PCL_LIBS_DIR}/libpcl_io_ply.dylib
#debug ${PCL_LIBS_DIR}/libpcl_2d.dylib
#debug /Users/williamnguatem/projects/libs/shapelib-1.3.0/libshp.a
#)

IF(NOT BUILD_SHARED_LIBS)
ADD_DEFINITIONS( -DANN_STATIC_LIB )
ENDIF()

ADD_LIBRARY (sc3dviewer ${simplecool3dviewer_HEADERS} ${simplecool3dviewer_SOURCES})

target_link_libraries(sc3dviewer ${EXTRA_LIBS} ${KML_LIB} ${PCLDebuglibs} ${GL3W} ${XERCES_LIB} ${ESRI_SHAPEFILE_LIB} ${OPENGL_LIBRARIES} ${GLFW_LIB} ${ASSIMP_LIB}) #${SOIL_LIB} ${PCL_LIBRARIES})#${OpenCV_LIBS} ${GLFW_LIB})
ENDIF(APPLE)
#		debug ${PCL_LIBRARIES}

IF(NOT BUILD_SHARED_LIBS)
ADD_DEFINITIONS( -DANN_STATIC_LIB )
ENDIF()

##add_executable (sc3dviewer main.cxx ${simplecool3dviewer_HEADERS} ${simplecool3dviewer_SOURCES})
ADD_LIBRARY (sc3dviewer ${simplecool3dviewer_HEADERS} ${simplecool3dviewer_SOURCES})
#target_link_libraries(sc3dviewer glfw ${OPENGL_LIBRARIES} ${ASSIMP_LIB} ${SOIL_LIB} ${GL3W})
##${OpenCV_LIBS} {GLFW_LIBRARIES})
target_link_libraries(sc3dviewer ${EXTRA_LIBS} ${KML_LIB} ${PCLDebuglibs} ${GL3W} ${XERCES_LIB} ${ESRI_SHAPEFILE_LIB} ${OPENGL_LIBRARIES} ${GLFW_LIB} ${ASSIMP_LIB}) #${SOIL_LIB} ${PCL_LIBRARIES})#${OpenCV_LIBS} ${GLFW_LIB})
# IF(WIN32)
#ADD_LIBRARY(sc3dviewer STATIC ${simplecool3dviewer_HEADERS} ${simplecool3dviewer_SOURCES} ${SOIL_LIB} ${KML_LIB} ${PCLDebuglibs} ${GL3W} ${XERCES_LIB} ${ESRI_SHAPEFILE_LIB} ${OPENGL_LIBRARIES} ${GLFW_LIB} ${ASSIMP_LIB})
#endif(WIN32)
#TESTING_CREATE_MODULE_CONF(ann
 # INCLUDE_DIRS include
#)
