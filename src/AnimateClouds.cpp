#include "MeshModelLoader.h"
#include "GameException.h"
#include "ColorHelper.h"
#include "Camera.h"
#include "Utility.h"
#include "ShaderProgram.h"
#include "VectorHelper.h"
#include "Model.h"
#include "ModelMaterial.h"
#include "Mesh.h"
#include "SOIL.h"
#include <sstream>
#include  <codecvt>
#include "Light.h"
#include "AnimateClouds.h"
#include <pcl/common/common.h>

#include <limits>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#ifdef __APPLE__
const std::string game_content = "/Users/williamnguatem/projects/LODViewer/game_content";
#endif

#ifdef WIN32
const std::string game_content = "C:/williamnguatem/Projects/Simple3DViewer/game_content";
#endif

using namespace glm;

namespace Rendering
{
    ////RTTI_DEFINITIONS(TexturedQuad)
    //
    //const std::string TexturedMeshloader::FilteringModeNames[] =
    //{
    //    "FilteringModePoint",
    //    "FilteringModeLinear",
    //    "FilteringModePointMipMapPoint",
    //    "FilteringModeLinearMipMapPoint",
    //    "FilteringModePointMipMapLinear",
    //    "FilteringModeTriLinear",
    //};
    
	AnimateClouds::AnimateClouds(Game& game, Camera& camera)
    : DrawableGameComponent(game, camera), mShaderProgram(), mVertexArrayObject(0), mVertexBuffer(0),
	mIndexBuffer(0), mWorldViewProjectionLocation(-1), mWorldMatrix(), mIndexCount(), mColorTexture(0), mActive_mode(0), mAlternative_mode(0),
    mTextureSamplers(), mTextureSamplersByFilteringMode(), mActiveFilteringMode(FilteringModePoint), mKeyboardHandler(nullptr), mAmbientLight(nullptr) 
    {
    }
    
	AnimateClouds::AnimateClouds()
    {
        DeleteObject(mAmbientLight);
        mGame->RemoveKeyboardHandler(mKeyboardHandler);
        glDeleteSamplers(mTextureSamplers.size(), &mTextureSamplers[0]);
        
        //for (size_t i = 0; i < 30; i++)
        //{
            //glDeleteTextures(1, &mColorTexture);
            glDeleteBuffers(1, &mIndexBuffer);
            glDeleteBuffers(1, &mVertexBuffer);
            //glDeleteVertexArrays(1, &mVertexArrayObject[i]);
//        }
    }
    
    
    void load_all_clouds_to_animate(const std::string &input_folder_path, std::vector<std::string> &cloud_filenames)
    {
        std::string ext(".pcd");
        
        boost::filesystem::path img_dir(input_folder_path);
        std::vector<std::string> img_file_names;
        if (!boost::filesystem::exists(img_dir)) return;
        if (boost::filesystem::is_directory(img_dir))
        {
            //boost::filesystem::recursive_directory_iterator it(root_postives);
            //boost::filesystem::recursive_directory_iterator endit;
            boost::filesystem::directory_iterator it(img_dir);
            boost::filesystem::directory_iterator endit;
            while (it != endit)
            {
                if (boost::filesystem::is_regular_file(*it) && (it->path().extension() == ext))
                {
                    img_file_names.push_back(it->path().string());
                }
                ++it;
            }
        }
        std::sort(img_file_names.begin(), img_file_names.end());
        cloud_filenames = img_file_names;
    }

    
    
	void AnimateClouds::Initialize()
    {
        // SetCurrentDirectory(Utility::ExecutableDirectory().c_str());
        
        //// Build the shader program
        //std::vector<ShaderDefinition> shaders;
        ////   shaders.push_back(ShaderDefinition(GL_VERTEX_SHADER, "FilteringModesDemo.vert"));
        ////   shaders.push_back(ShaderDefinition(GL_FRAGMENT_SHADER, "FilteringModesDemo.frag"));
        //shaders.push_back(ShaderDefinition(GL_VERTEX_SHADER, game_content+"/AmbientLightingDemo.vert"));
        //shaders.push_back(ShaderDefinition(GL_FRAGMENT_SHADER, game_content + "/AmbientLightingDemo.frag"));
        //mShaderProgram.BuildProgram(shaders);

		// Build the shader program
		std::vector<ShaderDefinition> shaders;
#ifdef __APPLE__
        shaders.push_back(ShaderDefinition(GL_VERTEX_SHADER, game_content + "/ModelDemo.vert"));
        shaders.push_back(ShaderDefinition(GL_FRAGMENT_SHADER, game_content + "/ModelDemo.frag"));

#endif
        
#ifdef WIN32
        shaders.push_back(ShaderDefinition(GL_VERTEX_SHADER, game_content + "/ModelDemo.vert"));
        shaders.push_back(ShaderDefinition(GL_FRAGMENT_SHADER, game_content + "/ModelDemo.frag"));

#endif
        
        
        
		mShaderProgram.BuildProgram(shaders);

        
        float size = 10.0f;
        float halfSize = size / 2.0f;
        
        //// Create the vertex buffer
        //VertexPositionTexture vertices[] =
        //{
        //	VertexPositionTexture(vec4(-halfSize, 1.0f, 0.0, 1.0f), vec2(0.0f, 1.0f)),
        //	VertexPositionTexture(vec4(-halfSize, size + 1.0f, 0.0f, 1.0f), vec2(0.0f, 0.0f)),
        //	VertexPositionTexture(vec4(halfSize, size + 1.0f, 0.0f, 1.0f), vec2(1.0f, 0.0f)),
        //	VertexPositionTexture(vec4(halfSize, 1.0f, 0.0f, 1.0f), vec2(1.0f, 1.0f))
        //};
        
		std::vector<VertexPositionColor> vertices;
        
        //std::string mesh_file("/Volumes/Volume/DataSet/LODGEN/trainingData/data101/3.obj");
        ////        std::string points_file("/Volumes/Volume/DataSet/LODGEN/trainingData/data101/3.point");
        //std::string g_dir("/Volumes/Volume/DataSet/LODGEN/trainingData/data101");
        
		//std::string g_dir("Q:/DataSet/wind_detector_from4/data100/1.obj");
		//std::string mesh_file("M:/DataSet/data19/1.obj");
	
		//        std::string points_file("/Volumes/Volume/DataSet/LODGEN/trainingData/data101/3.point");
        
        std::vector<std::string> animeFileNames;
        //load_all_clouds_to_animate("/Users/williamnguatem/DataSet/iccv_15/downsampled/gaus_sphere", animeFileNames);
        load_all_clouds_to_animate("Q:/DataSet/iccv_15/Ettlingen/patches_demo", animeFileNames);
        
        
        for(size_t i = 0; i < animeFileNames.size(); i++)
        {
            //std::string filename = "/Users/williamnguatem/DataSet/iccv_15/downsampled/haus_patches/patches_cloud_"+boost::lexical_cast<std::string>(i)+".pcd";
			std::string filename = "Q:/DataSet/iccv_15/Ettlingen/patches_demo/patches_cloud_"+boost::lexical_cast<std::string>(i)+".pcd";
            
            animeFileNames[i] = filename;
        }
        
        
        animeVertexArrayObject.resize(animeFileNames.size());
        animeVertexBuffer.resize(animeFileNames.size());
        animeIndexBuffer.resize(animeFileNames.size());
        animeIndexCount.resize(animeFileNames.size());
        
        for(size_t i = 0; i < animeFileNames.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            std::string mesh_file(animeFileNames[i]);
            pcl::io::loadPCDFile(mesh_file, *cloud);

            GLuint tmpvertexBuffer;
            CreateVertexBuffer(cloud, tmpvertexBuffer);
            animeVertexArrayObject[i] = tmpvertexBuffer;
            
            ////        mVertexBuffer.push_back(tmpvertexBuffer);
            GLuint tmpIdxBuffer;
            CreateIndexBuffer(cloud, tmpIdxBuffer);
            animeIndexBuffer[i] = tmpIdxBuffer;
            
            std::vector<GLuint> animeVertexBuffer;
            std::vector<GLuint> animeIndexBuffer;
            
            GLuint tmp_mVertexArrayObject;
            glGenVertexArrays(1, &tmp_mVertexArrayObject);
            glBindVertexArray(tmp_mVertexArrayObject);
            animeVertexArrayObject[i] = tmp_mVertexArrayObject;
            
            animeIndexCount[i] = cloud->points.size();
        //}
//        
//        
//        
//        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//        
//#ifdef __APPLE__
//	//std::string mesh_file("/Users/williamnguatem/projects/full3D_viewer/obj_samples/sibenik.obj");
//        std::string mesh_file("/Users/williamnguatem/DataSet/iccv_15/downsampled/gaus_sphere/out_sequence_0.pcd");
//        
//        pcl::io::loadPCDFile(mesh_file, *cloud);
//        
//        //std::string mesh_file ("/Users/williamnguatem/DataSet/iccv_15/downsampled/gaus_sphere/out_sequence_0.ply");
//#endif
//        
//#ifdef WIN32
//	std::string mesh_file("C:/williamnguatem/full3d_viewer/full3D_viewer/obj_samples/sibenik.ply");
//	//std::string mesh_file("Q:/DataSet/iccv_15/haus51_bonnland/mesh.ply");
//#endif
//	
//        // Load the model
//       // std::unique_ptr<Model> model(new Model(*mGame, mesh_file, true));
//        
//        //for (size_t i = 0; i < model->Meshes().size(); i++)
//        //{
//            // Create the vertex and index buffers
//         //   Mesh* mesh = model->Meshes().at(0); //assume only 1 mesh present
//            // Load the texture
//           // std::map<TextureType, std::vector<std::string>*> tmp_tex_file =  mesh->GetMaterial()->Textures();
//            //
//            //std::string text_filename ;
//            //for (std::pair<TextureType, std::vector<std::string>*> textures : tmp_tex_file)
//            //{
//            //    text_filename = textures.second->at(0);
//            //    //text_filename = *(textures.second);
//            //}
//
//			GLuint tmpvertexBuffer;
//			CreateVertexBuffer(cloud, tmpvertexBuffer);
//
////            
////            std::string img_filename = g_dir + "/" + text_filename ;
////            TexturedOBJMeshloader *tmp_obj_model = new TexturedOBJMeshloader(*mGame, *mCamera);
////            tmp_obj_model->setMesh(mesh);
////            tmp_obj_model->setTextureFile(img_filename);
////            //, *mesh, img_filename);
//////
////  //          TexturedOBJMeshloader tmp_obj_model(*mGame, mCamera, *mesh,  img_filename);
////			mOBJMeshModel.push_back(tmp_obj_model);
////   //         mOBJMeshModel.push_back(*tmp_obj_model);
////           // tmp_obj_model->Initialize();
//      //  }
////        
////        //initialization
////        for (size_t i = 0; i < mOBJMeshModel.size(); i++)
////        {
////			std::cout << "on initializing " << i << " of " << mOBJMeshModel.size() << " models" << std::endl;
////            mOBJMeshModel.at(i)->Initialize();
////        }
//////
//////        GLuint tmpvertexBuffer;
//////        CreateVertexBuffer(*mesh, tmpvertexBuffer);
//////        mVertexBuffer.push_back(tmpvertexBuffer);
//        GLuint tmpIdxBuffer;
//        CreateIndexBuffer(cloud, tmpIdxBuffer);
// //       mesh->;
//        //mIndexBuffer.push_back(tmpIdxBuffer);
//        //mIndexCount.push_back(mesh->Indices().size());
//
//
//		//GLuint indexBuffer;
//		//glGenBuffers(1, &indexBuffer);
//		//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
//		//glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * cloud_indices.size(), &cloud_indices[0], GL_STATIC_DRAW);
//		//mIndexBuffer = indexBuffer;
//		mIndexCount = cloud->points.size();
//
//		//CreateVertexBuffer(*cloud, mVertexBuffer);
//		////mesh->CreateIndexBuffer(mIndexBuffer);
//		////mIndexCount = mesh->Indices().size();
//		
//		mVertexBuffer = tmpvertexBuffer;
//		mIndexBuffer = tmpIdxBuffer;

		mActive_mode = GL_POINTS;
        mActiveIdx = 0;

		//glGenVertexArrays(1, &mVertexArrayObject);
		//glBindVertexArray(mVertexArrayObject);

		glVertexAttribPointer(VertexAttributePosition, 4, GL_FLOAT, GL_FALSE, sizeof(VertexPositionColor), (void*)offsetof(VertexPositionColor, Position));
		glEnableVertexAttribArray(VertexAttributePosition);

		glVertexAttribPointer(VertexAttributeColor, 4, GL_FLOAT, GL_FALSE, sizeof(VertexPositionColor), (void*)offsetof(VertexPositionColor, Color));
		glEnableVertexAttribArray(VertexAttributeColor);

		glBindVertexArray(0);

		mWorldViewProjectionLocation = glGetUniformLocation(mShaderProgram.Program(), "WorldViewProjection");
		if (mWorldViewProjectionLocation == -1)
		{
			throw GameException("glGetUniformLocation() did not find uniform location.");
		}

		// Attach the keyboard handler
		using namespace std::placeholders;
		mKeyboardHandler = std::bind(&AnimateClouds::OnKey, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
		mGame->AddKeyboardHandler(mKeyboardHandler);

////        
////        TexturedOBJMeshloader(Game& game, Camera& camera, const Mesh &meshData, const std::string &textureFilename)
////        
////        //        std::vector<std::string> img_file_names_;
////        //        readAllImages1(g_dir, img_file_names_);
////        //
////        std::vector<unsigned int> img_nr;
////        //        LoadFaces1(g_dir, mesh_file, m_faces, img_nr);
////        //
////        m_img_nr = img_nr;
////        //        //  m_img_nr.resize(1);
////        //        //img_nr.resize(1);
////        //        std::vector<unsigned int> ids;
////        //        std::vector<unsigned int> nr_p_img;
////        //        ReOrderVetexBuffer(m_faces, img_nr, nr_p_img, vertices, ids);
////        //        CreateVertexBuffer(vertices, nr_p_img, mVertexBuffer);
////        
////        //// Create the index buffer
////        //unsigned int indices[] =
////        //{
////        //	0, 2, 1,
////        //	0, 3, 2
////        //};
////        
////        //        //mIndexCount = sizeof(indices) / sizeof(unsigned int);
////        //        mIndexBuffer.resize(nr_p_img.size());
////        //        mIndexCount.resize(m_img_nr.size());
////        //        CreateIndexBuffer(ids, nr_p_img, mIndexBuffer);
////        
////        // Find the WVP uniform location
////        mWorldViewProjectionLocation = glGetUniformLocation(mShaderProgram.Program(), "WorldViewProjection");
////        if (mWorldViewProjectionLocation == -1)
////        {
////            throw GameException("glGetUniformLocation() did not find uniform location.");
////        }
////        
////        
////        mAmbientColorLocation = glGetUniformLocation(mShaderProgram.Program(), "AmbientColor");
////        if (mAmbientColorLocation == -1)
////        {
////            throw GameException("glGetUniformLocation() did not find uniform location.");
////        }
////        
////        std::string img_filename = game_content + "/EarthComposite.jpg";
////        
////        mColorTexture.resize(mIndexBuffer.size());
////        for (size_t i = 0; i < mIndexBuffer.size(); i++)
////        {
////            // Load the texture
////            std::map<TextureType, std::vector<std::string>*> tmp_tex_file =  mesh->GetMaterial()->Textures();
////            
////            std::string text_filename ;
////            for (std::pair<TextureType, std::vector<std::string>*> textures : tmp_tex_file)
////            {
////                text_filename = textures.second->at(0);
////                //text_filename = *(textures.second);
////            }
////            
////            std::string img_filename = g_dir + "/" + ws2s1(text_filename) ;
////            mColorTexture[i] = SOIL_load_OGL_texture(img_filename.c_str(), SOIL_LOAD_AUTO, SOIL_CREATE_NEW_ID, SOIL_FLAG_MIPMAPS | SOIL_FLAG_NTSC_SAFE_RGB);
////            if (mColorTexture[i] == 0)
////            {
////                throw GameException("SOIL_load_OGL_texture() failed.");
////            }
////            
////            //        int img_width1, img_height1;
////            //        unsigned char* img1 = SOIL_load_image("EarthComposite.jpg", &img_width1, &img_height1, NULL, 0);
////            //        glGenTextures(1, &mColorTexture);
////            //        glBindTexture(GL_TEXTURE_2D, mColorTexture);
////            //        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
////            //        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_width1, img_height1, 0, GL_RGB, GL_UNSIGNED_BYTE, img1);
////            
////        }
////        
////        
////        
////        
////        // Configure the texture samplers
////        mTextureSamplers.resize(FilteringModeEnd);
////        glGenSamplers(mTextureSamplers.size(), &mTextureSamplers[0]);
////        
////        for (FilteringMode mode = (FilteringMode)0; mode < FilteringModeEnd; mode = (FilteringMode)(mode + 1))
////        {
////            mTextureSamplersByFilteringMode[mode] = mTextureSamplers[mode];
////        }
////        
////        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModePoint], GL_TEXTURE_MIN_FILTER, GL_NEAREST);
////        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModePoint], GL_TEXTURE_MAG_FILTER, GL_NEAREST);
////        
////        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModeLinear], GL_TEXTURE_MIN_FILTER, GL_LINEAR);
////        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModeLinear], GL_TEXTURE_MAG_FILTER, GL_LINEAR);
////        
////        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModePointMipMapPoint], GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
////        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModePointMipMapPoint], GL_TEXTURE_MAG_FILTER, GL_NEAREST_MIPMAP_NEAREST);
////        
////        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModeLinearMipMapPoint], GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
////        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModeLinearMipMapPoint], GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_NEAREST);
////        
////        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModePointMipMapLinear], GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR);
////        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModePointMipMapLinear], GL_TEXTURE_MAG_FILTER, GL_NEAREST_MIPMAP_LINEAR);
////        
////        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModeTriLinear], GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
////        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModeTriLinear], GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);
////        
////        mVertexArrayObject.resize(mIndexBuffer.size());
////        
////        for (size_t i = 0; i < mIndexBuffer.size(); i++)
////        {
////            // Create the vertex array object
////            glGenVertexArrays(1, &mVertexArrayObject[i]);
////            glBindVertexArray(mVertexArrayObject[i]);
////            
////            glVertexAttribPointer(VertexAttributePosition, 4, GL_FLOAT, GL_FALSE, sizeof(VertexPositionTexture), (void*)offsetof(VertexPositionTexture, Position));
////            glEnableVertexAttribArray(VertexAttributePosition);
////            
////            glVertexAttribPointer(VertexAttributeTextureCoordinate, 2, GL_FLOAT, GL_FALSE, sizeof(VertexPositionTexture), (void*)offsetof(VertexPositionTexture, TextureCoordinates));
////            glEnableVertexAttribArray(VertexAttributeTextureCoordinate);
////        }
////        glBindVertexArray(0);
////        
////        // Attach the keyboard handler
////        using namespace std::placeholders;
////        mKeyboardHandler = std::bind(&TexturedMeshloader::OnKey, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
////        mGame->AddKeyboardHandler(mKeyboardHandler);
////        
////#if defined(DEBUG) || defined(_DEBUG)
////        OutputFilteringMode();
////#endif
////        
////        mAmbientLight = new Light(*mGame);
    }
    }
    
	void AnimateClouds::Update(const GameTime& gameTime)
    {
        ////UpdateAmbientLight(gameTime);
        ////initialization
        //for (size_t i = 0; i < mOBJMeshModel.size(); i++)
        //{
        //    mOBJMeshModel.at(i)->Update(gameTime);
        //}
    }
    
	void AnimateClouds::UpdateAmbientLight(const GameTime& gameTime)
    {
//        static float ambientIntensity = 1.0f;
//        //static float test_scalar = 0.10f;
//        
//        if (glfwGetKey(mGame->Window(), GLFW_KEY_L) && ambientIntensity < 1.0f)
//        {
//            ambientIntensity += (float)gameTime.ElapsedGameTime() / 50.0f;
//            ambientIntensity = min(ambientIntensity, 1.0f);
//            
//            mAmbientLight->SetColor(vec4((vec3)ambientIntensity, 1.0f));
//        }
//        
//        if (glfwGetKey(mGame->Window(), GLFW_KEY_O) && ambientIntensity > 0.0f)
//        {
//            ambientIntensity -= (float)gameTime.ElapsedGameTime() / 50.0f;
//            ambientIntensity = max(ambientIntensity, 0.0f);
//            
//            mAmbientLight->SetColor(vec4((vec3)ambientIntensity, 1.0f));
////        }
//        
//        for (size_t i = 0; i < mOBJMeshModel.size(); i++)
//        {
//            mOBJMeshModel.at(i)->UpdateAmbientLight(gameTime);
//        }
        
    }
    
    
	void AnimateClouds::Draw(const GameTime& gameTime)
    {

		glBindVertexArray(animeVertexArrayObject[mActiveIdx]);
        //glBindVertexArray(mVertexArrayObject);
		glBindBuffer(GL_ARRAY_BUFFER, animeVertexBuffer[mActiveIdx]);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, animeIndexBuffer[mActiveIdx]);

		mShaderProgram.Use();

		mat4 wvp = mCamera->ViewProjectionMatrix() * mWorldMatrix;
		glUniformMatrix4fv(mWorldViewProjectionLocation, 1, GL_FALSE, &wvp[0][0]);

		//glEnable(GL_POINT_SPRITE);
		//GL_POINT_SMOOTH;
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		glEnable(GL_CULL_FACE);
		glFrontFace(GL_CCW);

		//glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
		glPointSize(5.0f);
		//glDrawArrays(GL_POINTS, 0, mIndexCount);
		//glDrawArrays(GL_TRIANGLES, 0, mIndexCount);
		//glDrawArrays(GL_POINTS, 0, mIndexCount);
		glDrawElements(mActive_mode, animeIndexCount[mActiveIdx], GL_UNSIGNED_INT, 0);


        //for (size_t i = 0; i < mOBJMeshModel.size(); i++)
        //{
        //    mOBJMeshModel.at(i)->Draw(gameTime);
        //}
//
//        for (size_t i = 0; i <  mIndexBuffer.size(); i++)
//        {
//            glBindVertexArray(mVertexArrayObject[i]);
//            glBindBuffer(GL_ARRAY_BUFFER, mVertexBuffer[i]);
//            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndexBuffer[i]);
//            glBindTexture(GL_TEXTURE_2D, mColorTexture[i]);
//            
//            glBindSampler(0, mTextureSamplersByFilteringMode[mActiveFilteringMode]);
//            
//            glUseProgram(mShaderProgram.Program());
//            
//            mat4 wvp = mCamera->ViewProjectionMatrix() * mWorldMatrix;
//            glUniformMatrix4fv(mWorldViewProjectionLocation, 1, GL_FALSE, &wvp[0][0]);
//            glUniform4fv(mAmbientColorLocation, 1, &mAmbientLight->Color()[0]);
//            
//            glEnable(GL_CULL_FACE);
//            glFrontFace(GL_CCW);
//            
//            glDrawElements(GL_TRIANGLES, mIndexCount[i], GL_UNSIGNED_INT, 0);
//            //glDrawElements(GL_LINES, mIndexCount[i], GL_UNSIGNED_INT, 0);
//            glBindVertexArray(0);
//        }
		glBindVertexArray(0);
		//glDisableVertexAttribArray(0);
    }
    
	void AnimateClouds::CreateVertexBuffer(const std::vector<VertexPositionTexture> &vertices, const std::vector<unsigned int> &nr_ver_per_img, std::vector<GLuint>& vertexBuffer)
    {
        vertexBuffer.resize(m_img_nr.size());
        int ver_count_indx = 0;
        for (size_t i = 0; i < m_img_nr.size(); i++)
        {
            //GLuint vertexCount = sizeof(nr_ver_per_img.at(i)) / sizeof(VertexPositionTexture);
            GLuint vertexCount = nr_ver_per_img.at(i);// / sizeof(VertexPositionTexture);
            glGenBuffers(1, &vertexBuffer[i]);
            glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer[i]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPositionTexture) * vertexCount, &vertices[ver_count_indx], GL_STATIC_DRAW);
            ver_count_indx = ver_count_indx + nr_ver_per_img.at(i);
        }
    }
    
	void AnimateClouds::CreateIndexBuffer(const std::vector<unsigned int> &indices, const std::vector<unsigned int> &nr_ver_per_img, std::vector<GLuint>& indexBuffer)
    {
        //int ver_count_indx = 0;
        //mIndexCount.resize(m_img_nr.size());
        //for (size_t i = 0; i < m_img_nr.size(); i++)
        //{
        //    //GLuint indexCount = sizeof(nr_ver_per_img) / sizeof(unsigned int);
        //    GLuint indexCount = nr_ver_per_img.at(i);// / sizeof(unsigned int);
        //    glGenBuffers(1, &indexBuffer[i]);
        //    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer[i]);
        //    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * indexCount, &indices[ver_count_indx], GL_STATIC_DRAW);
        //    ver_count_indx = ver_count_indx + nr_ver_per_img.at(i);
        //    mIndexCount[i] = sizeof(unsigned int) * indexCount;
        //}
    }
    
    void AnimateClouds::CreateVertexBuffer(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, GLuint& tmpvertexBuffer) const
    {
		//std::vector<vec3> sourceVertices (cloud->points.size());

        Eigen::Vector4f max_val, min_val;
        pcl::getMinMax3D(*cloud, min_val, max_val);
        
		//fine the max value and shift s.t. the ground floor would be at y = 0;
		float min_y = std::numeric_limits<float>::max();
        min_y = min_val(1);
//		for (unsigned int i = 0; i < cloud->points.size(); i++)
//		{
//			vec3 position = sourceVertices.at(i);
//			if (position.y < min_y)
//			{
//				min_y = position.y;
//			}
//		}

		std::vector<VertexPositionColor> vertices;
		vertices.reserve(cloud->points.size());
		if (cloud->points.size() > 0)
		{
			for (unsigned int i = 0; i < cloud->points.size(); i++)
			{
                pcl::PointXYZRGBNormal tmp_point = cloud->points[i];
                float r = float(tmp_point.r)/255.0f;
                float g = float(tmp_point.g)/255.0f;
                float b = float(tmp_point.b)/255.0f;
				vertices.push_back(VertexPositionColor(vec4(tmp_point.x, tmp_point.y + abs(min_y), tmp_point.z, 1.0f),
                                                       vec4(r, g, b, 1.0f)));
			}
		}
		else
		{
//			for (unsigned int i = 0; i < sourceVertices.size(); i++)
//			{
//				vec3 position = sourceVertices.at(i);
//				vec4 color = ColorHelper::RandomColor();
//				vertices.push_back(VertexPositionColor(vec4(position.x, position.y + abs(min_y), position.z, 1.0f), color));
//			}
		}

		glGenBuffers(1, &tmpvertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, tmpvertexBuffer);
		glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPositionColor) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
    }
    
    void AnimateClouds::CreateIndexBuffer(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud, GLuint& indexBuffer) const
    {
        std::vector<unsigned int> cloud_ndices (cloud->points.size());
        for (size_t i = 0; i < cloud->points.size(); i++)
        {
            cloud_ndices[i] = i;
        }
        
        //std::cout<<mIndices.size()<<std::endl;
        glGenBuffers(1, &indexBuffer);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * cloud->points.size(), &cloud_ndices[0], GL_STATIC_DRAW);
    }
    
    
	void AnimateClouds::CreateVertexBuffer(VertexPositionTexture* vertices, GLuint vertexCount, GLuint& tmpvertexBuffer) const
    {
        glGenBuffers(1, &tmpvertexBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, tmpvertexBuffer);
        glBufferData(GL_ARRAY_BUFFER, VertexSize() * vertexCount, &vertices[0], GL_STATIC_DRAW);
    }
    
    
    
    
	unsigned int AnimateClouds::VertexSize() const
    {
		return sizeof(VertexPositionColor);
    }
    
    
    
	void AnimateClouds::OnKey(int key, int scancode, int action, int mods)
    {

		//if ((key == GLFW_KEY_M && action == GLFW_PRESS) && (key == GLFW_KEY_LEFT_CONTROL && action == GLFW_PRESS))
		if ((key == GLFW_KEY_M && action == GLFW_PRESS))
		{
//			if (mActive_mode == GL_POINTS)
//			{
//				mActive_mode = GL_LINES;
//				return;
//			}
//			if (mActive_mode == GL_LINES)
//			{
//				mActive_mode = GL_TRIANGLES;
//				return;
//			}
//			else
//			{
//				mActive_mode = GL_POINTS;
//			}
            mActiveIdx++;
		}

    }
    
	//void MeshModelLoader::OutputFilteringMode()
 //   {
	//	for (size_t i = 0; i < mOBJMeshModel.size(); i++)
	//	{
	//		mOBJMeshModel.at(i)->OutputFilteringMode();
	//	}
 //   }
}