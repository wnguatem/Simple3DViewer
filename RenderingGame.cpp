//#include "RenderingGame.h"
//#include "GameException.h"
//#include "FirstPersonCamera.h"
//#include "ColorHelper.h"
//#include "VectorHelper.h"
//#include "Grid.h"
//#include "TexturedModelDemo.h"
//
//using namespace glm;
//
//namespace Rendering
//{
//	////RTTI_DEFINITIONS(RenderingGame)
//
//	//RenderingGame::RenderingGame(HINSTANCE instance, const std::wstring& windowTitle)
//	//	:  Game(instance, windowTitle),
//	//	mCamera(nullptr), mKeyboardHandler(nullptr),
//	//	mTexturedModelDemo(nullptr)
//	//{
//	//	mDepthStencilBufferEnabled = true;
//	//}
//
//	RenderingGame::RenderingGame(const std::wstring& windowTitle)
//	: Game(windowTitle),
//	mCamera(nullptr), mKeyboardHandler(nullptr),
//	mTexturedModelDemo(nullptr)
//	{
//		mDepthStencilBufferEnabled = true;
//	}
//
//	void RenderingGame::Initialize()
//	{	
//
//		mCamera = new FirstPersonCamera(*this);
//		mComponents.push_back(mCamera);
//		//mServices.AddService(Camera::TypeIdClass(), mCamera);
//
//		using namespace std::placeholders;
//		mKeyboardHandler = std::bind(&RenderingGame::OnKey, this, _1, _2, _3, _4);
//		AddKeyboardHandler(mKeyboardHandler);
//
//		mGrid = new Grid(*this, *mCamera);
//		mComponents.push_back(mGrid);
//
//		mTexturedModelDemo = new TexturedModelDemo(*this, *mCamera);
//		mComponents.push_back(mTexturedModelDemo);
//
//		Game::Initialize();
//
//		mCamera->SetPosition(0, 5, 15);
//		mCamera->ApplyRotation(rotate(mat4(), 10.0f, Vector3Helper::Left));
//	}
//
//	void RenderingGame::Shutdown()
//	{
//		DeleteObject(mTexturedModelDemo);
//
//		DeleteObject(mGrid);
//		RemoveKeyboardHandler(mKeyboardHandler);
//		DeleteObject(mCamera);
//
//		Game::Shutdown();
//	}
//
//	void RenderingGame::Draw(const GameTime& gameTime)
//	{
//		static const GLfloat one = 1.0f;
//
//		glClearBufferfv(GL_COLOR, 0, &ColorHelper::CornflowerBlue[0]);
//		glClearBufferfv(GL_DEPTH, 0, &one);
//
//		Game::Draw(gameTime);
//
//		glfwSwapBuffers(mWindow);
//	}
//
//	void RenderingGame::OnKey(int key, int scancode, int action, int mods)
//	{
//		if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
//		{
//			Exit();
//		}
//	}
//}




#include "RenderingGame.h"
#include "GameException.h"
#include "FirstPersonCamera.h"
#include "ColorHelper.h"
#include "VectorHelper.h"
#include "Grid.h"
#include "TexturedModelDemo.h"
#include "VisPointCloud.h"
#include "QuadModeler.h"
#include "KMLModeler.h"

using namespace glm;

namespace Rendering
{
	//RTTI_DEFINITIONS(RenderingGame)

		RenderingGame::RenderingGame(const std::wstring& windowTitle)
		: Game(windowTitle),
		mCamera(nullptr), mKeyboardHandler(nullptr),
		mCubeDemo(nullptr), mPointDemo(nullptr)
	{
		mDepthStencilBufferEnabled = true;
	}

	void RenderingGame::Initialize()
	{
		mCamera = new FirstPersonCamera(*this);
		mComponents.push_back(mCamera);
		//mServices.AddService(Camera::TypeIdClass(), mCamera);

		using namespace std::placeholders;
		mKeyboardHandler = std::bind(&RenderingGame::OnKey, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
		AddKeyboardHandler(mKeyboardHandler);

		mGrid = new Grid(*this, *mCamera);
		mComponents.push_back(mGrid);

		//mCubeDemo = new CubeDemo(*this, *mCamera);
		//mComponents.push_back(mCubeDemo);
        
        mQuadModel = new QuadModeler(*this, *mCamera);
        mComponents.push_back(mQuadModel);
        
		//mPointDemo = new ModelDemo(*this, *mCamera);
		//mComponents.push_back(mPointDemo);

		mKMLModel = new KMLModeler(*this, *mCamera);
		mComponents.push_back(mKMLModel);

		Game::Initialize();

		mCamera->SetPosition(0, 5, 10);
		mCamera->ApplyRotation(rotate(mat4(), 30.0f, Vector3Helper::Left));
	}

	void RenderingGame::Shutdown()
	{
		DeleteObject(mCubeDemo);

		DeleteObject(mGrid);
		RemoveKeyboardHandler(mKeyboardHandler);
		DeleteObject(mCamera);

		Game::Shutdown();
	}

	void RenderingGame::Draw(const GameTime& gameTime)
	{
		static const GLfloat one = 1.0f;

		glClearBufferfv(GL_COLOR, 0, &ColorHelper::Black[0]);
		glClearBufferfv(GL_DEPTH, 0, &one);

		Game::Draw(gameTime);

		glfwSwapBuffers(mWindow);
	}

	void RenderingGame::OnKey(int key, int scancode, int action, int mods)
	{
		if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		{
			Exit();
		}
	}
}