#include "TexturedMeshloader.h"
#include "TexturedOBJMeshloader.h"
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

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

const std::wstring game_content = L"/Users/williamnguatem/projects/LODViewer/game_content";


std::wstring s2ws1(const std::string& str)
{
    typedef std::codecvt_utf8<wchar_t> convert_typeX;
    std::wstring_convert<convert_typeX, wchar_t> converterX;
    
    return converterX.from_bytes(str);
}

std::string ws2s1(const std::wstring& wstr)
{
    typedef std::codecvt_utf8<wchar_t> convert_typeX;
    std::wstring_convert<convert_typeX, wchar_t> converterX;
    
    return converterX.to_bytes(wstr);
}

using namespace glm;

namespace Rendering
{
    //RTTI_DEFINITIONS(TexturedQuad)
    
    const std::string TexturedMeshloader::FilteringModeNames[] =
    {
        "FilteringModePoint",
        "FilteringModeLinear",
        "FilteringModePointMipMapPoint",
        "FilteringModeLinearMipMapPoint",
        "FilteringModePointMipMapLinear",
        "FilteringModeTriLinear",
    };
    
    TexturedMeshloader::TexturedMeshloader(Game& game, Camera& camera)
    : DrawableGameComponent(game, camera), mShaderProgram(), mVertexArrayObject(0), mVertexBuffer(0),
    mIndexBuffer(0), mWorldViewProjectionLocation(-1), mWorldMatrix(), mIndexCount(), mColorTexture(0),
    mTextureSamplers(), mTextureSamplersByFilteringMode(), mActiveFilteringMode(FilteringModePoint), mKeyboardHandler(nullptr), mAmbientLight(nullptr), mOBJMeshModel(0)
    {
    }
    
    TexturedMeshloader::~TexturedMeshloader()
    {
        DeleteObject(mAmbientLight);
        mGame->RemoveKeyboardHandler(mKeyboardHandler);
        glDeleteSamplers(mTextureSamplers.size(), &mTextureSamplers[0]);
        
        for (size_t i = 0; i < 30; i++)
        {
            glDeleteTextures(1, &mColorTexture[i]);
            glDeleteBuffers(1, &mIndexBuffer[i]);
            glDeleteBuffers(1, &mVertexBuffer[i]);
            //glDeleteVertexArrays(1, &mVertexArrayObject[i]);
        }
    }
    
    void TexturedMeshloader::Initialize()
    {
        // SetCurrentDirectory(Utility::ExecutableDirectory().c_str());
        
        // Build the shader program
        std::vector<ShaderDefinition> shaders;
        //   shaders.push_back(ShaderDefinition(GL_VERTEX_SHADER, L"FilteringModesDemo.vert"));
        //   shaders.push_back(ShaderDefinition(GL_FRAGMENT_SHADER, L"FilteringModesDemo.frag"));
        shaders.push_back(ShaderDefinition(GL_VERTEX_SHADER, game_content+L"/AmbientLightingDemo.vert"));
        shaders.push_back(ShaderDefinition(GL_FRAGMENT_SHADER, game_content + L"/AmbientLightingDemo.frag"));
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
        
        std::vector<VertexPositionTexture> vertices;
        
        std::string mesh_file("/Volumes/Volume/DataSet/LODGEN/trainingData/data101/3.obj");
        //        std::string points_file("/Volumes/Volume/DataSet/LODGEN/trainingData/data101/3.point");
        std::string g_dir("/Volumes/Volume/DataSet/LODGEN/trainingData/data101");
        
        // Load the model
        std::unique_ptr<Model> model(new Model(*mGame, mesh_file, true));
        
        for (size_t i = 0; i < model->Meshes().size(); i++)
        {
            // Create the vertex and index buffers
            Mesh* mesh = model->Meshes().at(i);
            // Load the texture
            std::map<TextureType, std::vector<std::wstring>*> tmp_tex_file =  mesh->GetMaterial()->Textures();
            
            std::wstring text_filename ;
            for (std::pair<TextureType, std::vector<std::wstring>*> textures : tmp_tex_file)
            {
                text_filename = textures.second->at(0);
                //text_filename = *(textures.second);
            }
            
            std::string img_filename = g_dir + "/" + ws2s1(text_filename) ;
            TexturedOBJMeshloader *tmp_obj_model = new TexturedOBJMeshloader(*mGame, *mCamera);
            //tmp_obj_model->setMesh(mesh);
            //tmp_obj_model->setTextureFile(img_filename);
            //, *mesh, img_filename);
//
  //          TexturedOBJMeshloader tmp_obj_model(*mGame, mCamera, *mesh,  img_filename);
          //  mOBJMeshModel.push_back(*tmp_obj_model);
           // tmp_obj_model->Initialize();
        }
        
//        //initialization
//        for (size_t i = 0; i < mOBJMeshModel.size(); i++)
//        {
//            mOBJMeshModel.at(i).Initialize();
//        }
//
//        GLuint tmpvertexBuffer;
//        CreateVertexBuffer(*mesh, tmpvertexBuffer);
//        mVertexBuffer.push_back(tmpvertexBuffer);
//        GLuint tmpIdxBuffer;
//        mesh->CreateIndexBuffer(tmpIdxBuffer);
//        mIndexBuffer.push_back(tmpIdxBuffer);
//        mIndexCount.push_back(mesh->Indices().size());
//        
//        TexturedOBJMeshloader(Game& game, Camera& camera, const Mesh &meshData, const std::string &textureFilename)
//        
//        //        std::vector<std::string> img_file_names_;
//        //        readAllImages1(g_dir, img_file_names_);
//        //
//        std::vector<unsigned int> img_nr;
//        //        LoadFaces1(g_dir, mesh_file, m_faces, img_nr);
//        //
//        m_img_nr = img_nr;
//        //        //  m_img_nr.resize(1);
//        //        //img_nr.resize(1);
//        //        std::vector<unsigned int> ids;
//        //        std::vector<unsigned int> nr_p_img;
//        //        ReOrderVetexBuffer(m_faces, img_nr, nr_p_img, vertices, ids);
//        //        CreateVertexBuffer(vertices, nr_p_img, mVertexBuffer);
//        
//        //// Create the index buffer
//        //unsigned int indices[] =
//        //{
//        //	0, 2, 1,
//        //	0, 3, 2
//        //};
//        
//        //        //mIndexCount = sizeof(indices) / sizeof(unsigned int);
//        //        mIndexBuffer.resize(nr_p_img.size());
//        //        mIndexCount.resize(m_img_nr.size());
//        //        CreateIndexBuffer(ids, nr_p_img, mIndexBuffer);
//        
//        // Find the WVP uniform location
//        mWorldViewProjectionLocation = glGetUniformLocation(mShaderProgram.Program(), "WorldViewProjection");
//        if (mWorldViewProjectionLocation == -1)
//        {
//            throw GameException("glGetUniformLocation() did not find uniform location.");
//        }
//        
//        
//        mAmbientColorLocation = glGetUniformLocation(mShaderProgram.Program(), "AmbientColor");
//        if (mAmbientColorLocation == -1)
//        {
//            throw GameException("glGetUniformLocation() did not find uniform location.");
//        }
//        
//        std::wstring img_filename = game_content + L"/EarthComposite.jpg";
//        
//        mColorTexture.resize(mIndexBuffer.size());
//        for (size_t i = 0; i < mIndexBuffer.size(); i++)
//        {
//            // Load the texture
//            std::map<TextureType, std::vector<std::wstring>*> tmp_tex_file =  mesh->GetMaterial()->Textures();
//            
//            std::wstring text_filename ;
//            for (std::pair<TextureType, std::vector<std::wstring>*> textures : tmp_tex_file)
//            {
//                text_filename = textures.second->at(0);
//                //text_filename = *(textures.second);
//            }
//            
//            std::string img_filename = g_dir + "/" + ws2s1(text_filename) ;
//            mColorTexture[i] = SOIL_load_OGL_texture(img_filename.c_str(), SOIL_LOAD_AUTO, SOIL_CREATE_NEW_ID, SOIL_FLAG_MIPMAPS | SOIL_FLAG_NTSC_SAFE_RGB);
//            if (mColorTexture[i] == 0)
//            {
//                throw GameException("SOIL_load_OGL_texture() failed.");
//            }
//            
//            //        int img_width1, img_height1;
//            //        unsigned char* img1 = SOIL_load_image("EarthComposite.jpg", &img_width1, &img_height1, NULL, 0);
//            //        glGenTextures(1, &mColorTexture);
//            //        glBindTexture(GL_TEXTURE_2D, mColorTexture);
//            //        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//            //        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_width1, img_height1, 0, GL_RGB, GL_UNSIGNED_BYTE, img1);
//            
//        }
//        
//        
//        
//        
//        // Configure the texture samplers
//        mTextureSamplers.resize(FilteringModeEnd);
//        glGenSamplers(mTextureSamplers.size(), &mTextureSamplers[0]);
//        
//        for (FilteringMode mode = (FilteringMode)0; mode < FilteringModeEnd; mode = (FilteringMode)(mode + 1))
//        {
//            mTextureSamplersByFilteringMode[mode] = mTextureSamplers[mode];
//        }
//        
//        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModePoint], GL_TEXTURE_MIN_FILTER, GL_NEAREST);
//        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModePoint], GL_TEXTURE_MAG_FILTER, GL_NEAREST);
//        
//        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModeLinear], GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModeLinear], GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//        
//        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModePointMipMapPoint], GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
//        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModePointMipMapPoint], GL_TEXTURE_MAG_FILTER, GL_NEAREST_MIPMAP_NEAREST);
//        
//        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModeLinearMipMapPoint], GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
//        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModeLinearMipMapPoint], GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_NEAREST);
//        
//        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModePointMipMapLinear], GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR);
//        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModePointMipMapLinear], GL_TEXTURE_MAG_FILTER, GL_NEAREST_MIPMAP_LINEAR);
//        
//        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModeTriLinear], GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
//        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModeTriLinear], GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);
//        
//        mVertexArrayObject.resize(mIndexBuffer.size());
//        
//        for (size_t i = 0; i < mIndexBuffer.size(); i++)
//        {
//            // Create the vertex array object
//            glGenVertexArrays(1, &mVertexArrayObject[i]);
//            glBindVertexArray(mVertexArrayObject[i]);
//            
//            glVertexAttribPointer(VertexAttributePosition, 4, GL_FLOAT, GL_FALSE, sizeof(VertexPositionTexture), (void*)offsetof(VertexPositionTexture, Position));
//            glEnableVertexAttribArray(VertexAttributePosition);
//            
//            glVertexAttribPointer(VertexAttributeTextureCoordinate, 2, GL_FLOAT, GL_FALSE, sizeof(VertexPositionTexture), (void*)offsetof(VertexPositionTexture, TextureCoordinates));
//            glEnableVertexAttribArray(VertexAttributeTextureCoordinate);
//        }
//        glBindVertexArray(0);
//        
//        // Attach the keyboard handler
//        using namespace std::placeholders;
//        mKeyboardHandler = std::bind(&TexturedMeshloader::OnKey, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
//        mGame->AddKeyboardHandler(mKeyboardHandler);
//        
//#if defined(DEBUG) || defined(_DEBUG)
//        OutputFilteringMode();
//#endif
//        
//        mAmbientLight = new Light(*mGame);
    }
    
    void TexturedMeshloader::Update(const GameTime& gameTime)
    {
        //UpdateAmbientLight(gameTime);
        //initialization
        for (size_t i = 0; i < mOBJMeshModel.size(); i++)
        {
         //   mOBJMeshModel.at(i).Update(gameTime);
        }
    }
    
    void TexturedMeshloader::UpdateAmbientLight(const GameTime& gameTime)
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
//        }
        
        for (size_t i = 0; i < mOBJMeshModel.size(); i++)
        {
           // mOBJMeshModel.at(i).UpdateAmbientLight(gameTime);
        }
        
    }
    
    
    void TexturedMeshloader::Draw(const GameTime& gameTime)
    {
        for (size_t i = 0; i < mOBJMeshModel.size(); i++)
        {
           // mOBJMeshModel.at(i).Draw(gameTime);
        }
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
//        //glBindVertexArray(0);
//        //glDisableVertexAttribArray(0);
    }
    
    void TexturedMeshloader::CreateVertexBuffer(const std::vector<VertexPositionTexture> &vertices, const std::vector<unsigned int> &nr_ver_per_img, std::vector<GLuint>& vertexBuffer)
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
    
    void TexturedMeshloader::CreateIndexBuffer(const std::vector<unsigned int> &indices, const std::vector<unsigned int> &nr_ver_per_img, std::vector<GLuint>& indexBuffer)
    {
        int ver_count_indx = 0;
        mIndexCount.resize(m_img_nr.size());
        for (size_t i = 0; i < m_img_nr.size(); i++)
        {
            //GLuint indexCount = sizeof(nr_ver_per_img) / sizeof(unsigned int);
            GLuint indexCount = nr_ver_per_img.at(i);// / sizeof(unsigned int);
            glGenBuffers(1, &indexBuffer[i]);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer[i]);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * indexCount, &indices[ver_count_indx], GL_STATIC_DRAW);
            ver_count_indx = ver_count_indx + nr_ver_per_img.at(i);
            mIndexCount[i] = sizeof(unsigned int) * indexCount;
        }
    }
    
    void TexturedMeshloader::CreateVertexBuffer(const Mesh& mesh, GLuint& tmpvertexBuffer) const
    {
        const std::vector<vec3>& sourceVertices = mesh.Vertices();
        
        std::vector<VertexPositionTexture> vertices;
        vertices.reserve(sourceVertices.size());
        
        std::vector<vec3>* textureCoordinates = mesh.TextureCoordinates().at(0);
        assert(textureCoordinates->size() == sourceVertices.size());
        
        const std::vector<vec3>& normals = mesh.Normals();
        //assert(normals.size() == sourceVertices.size());
        
        for (unsigned int i = 0; i < sourceVertices.size(); i++)
        {
            vec3 position = sourceVertices.at(i);
            vec2 uv = (vec2)textureCoordinates->at(i);
            //vec3 normal = normals.at(i);
            vertices.push_back(VertexPositionTexture(vec4(position.x, position.y, position.z, 1.0f), uv));
        }
        
        CreateVertexBuffer(&vertices[0], vertices.size(), tmpvertexBuffer);
    }
    
    void TexturedMeshloader::CreateVertexBuffer(VertexPositionTexture* vertices, GLuint vertexCount, GLuint& tmpvertexBuffer) const
    {
        glGenBuffers(1, &tmpvertexBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, tmpvertexBuffer);
        glBufferData(GL_ARRAY_BUFFER, VertexSize() * vertexCount, &vertices[0], GL_STATIC_DRAW);
    }
    
    unsigned int TexturedMeshloader::VertexSize() const
    {
        return sizeof(VertexPositionTexture);
    }
    
    
    
    void TexturedMeshloader::OnKey(int key, int scancode, int action, int mods)
    {
//        if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
//        {
//            FilteringMode activeMode = FilteringMode(mActiveFilteringMode + 1);
//            if (activeMode >= FilteringModeEnd)
//            {
//                activeMode = (FilteringMode)(0);
//            }
//            
//            mActiveFilteringMode = activeMode;
//            
//#if defined(DEBUG) || defined(_DEBUG)
//            OutputFilteringMode();
//#endif
//        }
    }
    
    void TexturedMeshloader::OutputFilteringMode()
    {
//        std::wostringstream message;
//        message << "Active Filtering Mode: " << FilteringModeNames[mActiveFilteringMode].c_str() << "\n";
//        //OutputDebugString(message.str().c_str());
    }
}