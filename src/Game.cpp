#include "Game.h"
#include "DrawableGameComponent.h"
#include "GameException.h"
#include "Utility.h"
#include "GLFW/glfw3native.h"

namespace Library
{
	////////RTTI_DEFINITIONS(Game)

	const unsigned int Game::DefaultScreenWidth =  800;// 720;// 800;
	const unsigned int Game::DefaultScreenHeight = 600;// 1280;// 600;

	Game* Game::sInternalInstance = nullptr;

	//Game::Game(HINSTANCE instance, const std::string& windowTitle)
	//	: mInstance(instance), mWindow(nullptr), mWindowTitle(windowTitle),
	//	  mScreenWidth(DefaultScreenWidth), mScreenHeight(DefaultScreenHeight), mIsFullScreen(false),		  
	//	  mMajorVersion(0), mMinorVersion(0),
	//	  mGameClock(), mGameTime(), mComponents(), mServices(), mKeyboardHandlers(),
	//	  mDepthStencilBufferEnabled(false)
	//{
	//	GlobalServices.AddService(TypeIdClass(), &(*this));
	//}

	Game::Game(const std::string& windowTitle)
		: mWindow(nullptr), mWindowTitle(windowTitle),
		mScreenWidth(DefaultScreenWidth), mScreenHeight(DefaultScreenHeight), mIsFullScreen(false),
		mMajorVersion(0), mMinorVersion(0),
		mGameClock(), mGameTime(), mComponents(), mServices(), mKeyboardHandlers(),
		mDepthStencilBufferEnabled(false)
	{
		//GlobalServices.AddService(TypeIdClass(), &(*this));
	}

	Game::~Game()
	{
		mComponents.clear();
	}

	//HINSTANCE Game::Instance() const
	//{
	//	return mInstance;
	//}

	GLFWwindow* Game::Window() const
	{
		return mWindow;
	}

	//HWND Game::WindowHandle() const
	//{
	//	return glfwGetWin32Window(mWindow);
	//}

	const std::string& Game::WindowTitle() const
	{
		return mWindowTitle;
	}

	int Game::ScreenWidth() const
	{
		return mScreenWidth;
	}

	int Game::ScreenHeight() const
	{
		return mScreenHeight;
	}

	float Game::AspectRatio() const
	{
		return static_cast<float>(mScreenWidth) / mScreenHeight;
	}
	
	bool Game::IsFullScreen() const
	{
		return mIsFullScreen;
	}

	const std::vector<GameComponent*>& Game::Components() const
	{
		return mComponents;
	}

	const ServiceContainer& Game::Services() const
	{
		return mServices;
	}

	void Game::Run()
	{
		sInternalInstance = this;

		InitializeWindow();
		InitializeOpenGL();
		Initialize();

		mGameClock.Reset();

		while (!glfwWindowShouldClose(mWindow))
		{
			setViewPort();
			mGameClock.UpdateGameTime(mGameTime);
			Update(mGameTime);
			Draw(mGameTime);

			glfwPollEvents();
		}

		Shutdown();
	}

	void  Game::setViewPort()
	{
		int win_width = 0, win_height = 0;
		glfwGetFramebufferSize(mWindow, &win_width, &win_height);
		glViewport(0, 0, win_width, win_height);
		mScreenWidth = win_width;
		mScreenHeight = win_height;
	}


	void Game::Exit()
	{
		glfwSetWindowShouldClose(mWindow, GL_TRUE);
	}

	void Game::Initialize()
	{
		for (GameComponent* component : mComponents)
		{
			component->Initialize();
		}
	}

	void Game::Update(const GameTime& gameTime)
	{
		for (GameComponent* component : mComponents)
		{
			if (component->Enabled())
			{
				component->Update(gameTime);
			}
		}
	}

	void Game::Draw(const GameTime& gameTime)
	{
		for (GameComponent* component : mComponents)
		{
			//DrawableGameComponent* drawableGameComponent = component->As<DrawableGameComponent>();
			//if (drawableGameComponent != nullptr && drawableGameComponent->Visible())
			//{
			//	drawableGameComponent->Draw(gameTime);
			//}

			DrawableGameComponent* drawableGameComponent = dynamic_cast<DrawableGameComponent*>(component);
			if (drawableGameComponent != nullptr && drawableGameComponent->Visible())
			{
				drawableGameComponent->Draw(gameTime);
			}
		}
	}

	void Game::AddKeyboardHandler(KeyboardHandler handler)
	{
		mKeyboardHandlers[&handler] = handler;
	}

	void Game::RemoveKeyboardHandler(KeyboardHandler handler)
	{
		mKeyboardHandlers.erase(&handler);
	}

	void Game::InitializeWindow()
	{
//		if (!glfwInit())
//		{
//			throw GameException("glfwInit() failed.");
//		}
//
//		//mIsFullScreen = true;
//
//		GLFWmonitor* monitor = (mIsFullScreen ? glfwGetPrimaryMonitor() : nullptr);
//		mWindow = glfwCreateWindow(mScreenWidth, mScreenHeight, Utility::ToString(mWindowTitle).c_str(), monitor, nullptr);
//		if (mWindow == nullptr)
//		{
//			Shutdown();
//			throw GameException("glfwCreateWindow() failed.");
//		}
//
//		POINT1 center = CenterWindow(mScreenWidth, mScreenHeight);
//		glfwSetWindowPos(mWindow, center.x, center.y);
        
        
        
        
        
        
        if (!glfwInit())
        {
            throw GameException("glfwInit() failed.");
        }
        
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
        
        GLFWmonitor* monitor = (mIsFullScreen ? glfwGetPrimaryMonitor() : nullptr);
        mWindow = glfwCreateWindow(mScreenWidth, mScreenHeight, Utility::ToString(mWindowTitle).c_str(), monitor, nullptr);
        if (mWindow == nullptr)
        {
            Shutdown();
            throw GameException("glfwCreateWindow() failed.");
        }
        
        POINT1 center = CenterWindow(mScreenWidth, mScreenHeight);
        glfwSetWindowPos(mWindow, center.x, center.y);
        

	}

	void Game::InitializeOpenGL()
	{
		glfwMakeContextCurrent(mWindow);
		
		if (gl3wInit() != 0)
		{
			throw GameException("gl3wInit() failed.");
		}

		glGetIntegerv(GL_MAJOR_VERSION, &mMajorVersion);
		glGetIntegerv(GL_MINOR_VERSION, &mMinorVersion);

		if (mDepthStencilBufferEnabled)
		{
			//set the stencil's clear value
			//glClearStencil(0x0);
			glEnable(GL_DEPTH_TEST);
			//glEnable(GL_STENCIL_TEST);
			glDepthFunc(GL_LEQUAL);
		}

		glViewport(0, 0, mScreenWidth, mScreenHeight);

		glfwSetKeyCallback(mWindow, Game::OnKey);
	}

	void Game::Shutdown()
	{
		glfwDestroyWindow(mWindow);
		glfwTerminate();
	}

	void Game::OnKey(GLFWwindow* window, int key, int scancode, int action, int mods)
	{
		for (auto handler : sInternalInstance->mKeyboardHandlers)
		{
			handler.second(key, scancode, action, mods);
		}
	}
	
	POINT1 Game::CenterWindow(int windowWidth, int windowHeight)
	{
		//GLFWvidmode return_struct;

		//glfwGetDesktopMode(&return_struct);

		//int height = return_struct.Height;


		const GLFWvidmode * mode = glfwGetVideoMode(glfwGetPrimaryMonitor());

		int window_width = mode->width;
		int window_height = mode->height;
		
		POINT1 center;
		center.x = (window_width - windowWidth) / 2;
		center.y = (window_height - windowHeight) / 2;

		return center;
	}	
}