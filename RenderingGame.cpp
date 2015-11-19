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
#include "TexturedQuad.h"
#include "TexturedMeshloader.h"
#include "TexturedOBJMeshloader.h"
#include "VisPointCloud.h"
#include "QuadModeler.h"
#include "KMLModeler.h"
#include "Quad.h"
#include "Quads.h"
#include "SpotlightDemo.h"
#include "NormalMappingDemo.h"
#include "TransparencyMappingDemo.h"
#include "EnvironmentMappingDemo.h"
#include "Skybox.h"
#include "FogDemo.h"
#include "LineStrip.h"


using namespace glm;

namespace Rendering
{
	//RTTI_DEFINITIONS(RenderingGame)

		RenderingGame::RenderingGame(const std::wstring& windowTitle)
        : Game(windowTitle),m_input_data(""),
		mCamera(nullptr), mKeyboardHandler(nullptr),
		mCubeDemo(nullptr), mPointDemo(nullptr), mTransparencyMappingDemo(nullptr), mEnvironmentMappingDemo(nullptr), mSkybox(nullptr), mFogDemo(nullptr), mGameContentFolder(L"")
	{
        mDepthStencilBufferEnabled = true;
	}
    
    std::string RenderingGame::getInputData()
    {
        return m_input_data;
    }

	std::wstring RenderingGame::getGameContentFolder()
	{
		return mGameContentFolder;
	}
    
    void RenderingGame::setInputData(const std::string &input_data)
    {
        m_input_data = input_data;
    }
    
    void RenderingGame::addQuad(const pcl::PointCloud<pcl::PointXYZ>::Ptr &quad_cloud, const glm::vec4 &quad_color)
    {
        m_quad_cloud = quad_cloud;
        m_quad_color = quad_color;
    }

	void RenderingGame::Initialize()
	{
		//get the current dir
		//boost::filesystem::path full_path(boost::filesystem::current_path());
		//std::cout << "Current path is : " << full_path << std::endl;
		//mGameContentFolder = full_path.wstring() + L"\\game_content";

		//mGameContentFolder =  L"/Users/williamnguatem/projects/LODViewer/game_content";
		mGameContentFolder = L"C:/williamnguatem/Projects/Simple3DViewer/game_content";

		mCamera = new FirstPersonCamera(*this);
		mComponents.push_back(mCamera);
		//mServices.AddService(Camera::TypeIdClass(), mCamera);

		using namespace std::placeholders;
		mKeyboardHandler = std::bind(&RenderingGame::OnKey, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
		AddKeyboardHandler(mKeyboardHandler);

		mGrid = new Grid(*this, *mCamera);
		mComponents.push_back(mGrid);

		lodgenMesh = new TexturedMeshloader(*this, *mCamera);
		mComponents.push_back(lodgenMesh);

        lodgenObjMesh = new TexturedOBJMeshloader(*this, *mCamera);
        //mComponents.push_back(lodgenObjMesh);

        

		mTexturedQuad = new TexturedQuad(*this, *mCamera);
		//mComponents.push_back(mTexturedQuad);
        
        mTexturedQuad = new TexturedQuad(*this, *mCamera);
        //mComponents.push_back(mTexturedQuad);

		//mCubeDemo = new CubeDemo(*this, *mCamera);
		//mComponents.push_back(mCubeDemo);
        
//        mQuadModel = new QuadModeler(*this, *mCamera);
//        mComponents.push_back(mQuadModel);
//        
		mPointDemo = new ModelDemo(*this, *mCamera);
        //mPointDemo->setInputData(m_input_data);
		//mComponents.push_back(mPointDemo);

		mCubeDemo = new CubeDemo(*this, *mCamera);
		//mComponents.push_back(mCubeDemo);
//
//		mKMLModel = new KMLModeler(*this, *mCamera);
//		mComponents.push_back(mKMLModel);

       // mQuad = new Quad(*this, *mCamera, m_quad_cloud, m_quad_color);
        //if(mQuad!=nullptr)
       // mComponents.push_back(mQuad);
        
       // mSpotLightDemo = new SpotLightDemo(*this, *mCamera);
       // mComponents.push_back(mSpotLightDemo);
        
        mFacadeFileModeler = new FacadeFileModeler(*this, *mCamera);
       // mComponents.push_back(mFacadeFileModeler);
        
      mSkybox = new Skybox(*this, *mCamera, "Maskonaive2_1024/posx.jpg", "Maskonaive2_1024/negx.jpg", "Maskonaive2_1024/posy.jpg", "Maskonaive2_1024/negy.jpg", "Maskonaive2_1024/posz.jpg", "Maskonaive2_1024/negz.jpg", 100.0f);
       //mComponents.push_back(mSkybox);
        
        mFogDemo = new FogDemo(*this, *mCamera);
        //mComponents.push_back(mFogDemo);
        
        mNormalMappingDemo = new NormalMappingDemo(*this, *mCamera);
        //mComponents.push_back(mNormalMappingDemo);
        
        mTransparencyMappingDemo = new TransparencyMappingDemo(*this, *mCamera);
        //mComponents.push_back(mTransparencyMappingDemo);
        
        mEnvironmentMappingDemo = new EnvironmentMappingDemo(*this, *mCamera);
        //mComponents.push_back(mEnvironmentMappingDemo);
		Game::Initialize();

		mCamera->SetPosition(0, 5, 10);
		mCamera->ApplyRotation(rotate(mat4(), 30.0f, Vector3Helper::Left));
	}

	void RenderingGame::Shutdown()
	{
		DeleteObject(mCubeDemo);
        DeleteObject(mTransparencyMappingDemo);
        DeleteObject(mEnvironmentMappingDemo);
        DeleteObject(mSkybox)

		DeleteObject(mGrid);
		RemoveKeyboardHandler(mKeyboardHandler);
		DeleteObject(mCamera);

		Game::Shutdown();
	}

	void RenderingGame::Draw(const GameTime& gameTime)
	{
//		static const GLfloat one = 1.0f;
//
//		glClearBufferfv(GL_COLOR, 0, &ColorHelper::Black[0]);
//		glClearBufferfv(GL_DEPTH, 0, &one);
//
//		Game::Draw(gameTime);
//
//		glfwSwapBuffers(mWindow);
        
        static const GLfloat one = 1.0f;
        
        glClearBufferfv(GL_COLOR, 0, &ColorHelper::CornflowerBlue[0]);
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