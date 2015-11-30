#include "TexturedQuad.h"
#include "GameException.h"
#include "ColorHelper.h"
#include "Camera.h"
#include "Utility.h"
#include "ShaderProgram.h"
#include "VectorHelper.h"
#include "Model.h"
#include "Mesh.h"
#include "SOIL.h"
#include <sstream>
#include "Light.h"

using namespace glm;

namespace Rendering
{
    //RTTI_DEFINITIONS(TexturedQuad)
    
    const std::string TexturedQuad::FilteringModeNames[] =
    {
        "FilteringModePoint",
        "FilteringModeLinear",
        "FilteringModePointMipMapPoint",
        "FilteringModeLinearMipMapPoint",
        "FilteringModePointMipMapLinear",
        "FilteringModeTriLinear",
    };
    
    TexturedQuad::TexturedQuad(Game& game, Camera& camera)
    : DrawableGameComponent(game, camera), mShaderProgram(), mVertexArrayObject(0), mVertexBuffer(0),
    mIndexBuffer(0), mWorldViewProjectionLocation(-1), mWorldMatrix(), mIndexCount(), mColorTexture(0),
    mTextureSamplers(), mTextureSamplersByFilteringMode(), mActiveFilteringMode(FilteringModePoint), mKeyboardHandler(nullptr),mAmbientLight(nullptr)
    {
    }
    
    TexturedQuad::~TexturedQuad()
    {
        DeleteObject(mAmbientLight);
        mGame->RemoveKeyboardHandler(mKeyboardHandler);
        glDeleteSamplers(mTextureSamplers.size(), &mTextureSamplers[0]);
        glDeleteTextures(1, &mColorTexture);
        glDeleteBuffers(1, &mIndexBuffer);
        glDeleteBuffers(1, &mVertexBuffer);
        glDeleteVertexArrays(1, &mVertexArrayObject);
    }
    
    void TexturedQuad::Initialize()
    {
        // SetCurrentDirectory(Utility::ExecutableDirectory().c_str());
        
        // Build the shader program
        std::vector<ShaderDefinition> shaders;
     //   shaders.push_back(ShaderDefinition(GL_VERTEX_SHADER, "FilteringModesDemo.vert"));
     //   shaders.push_back(ShaderDefinition(GL_FRAGMENT_SHADER, "FilteringModesDemo.frag"));
		const std::string game_content = "C:/williamnguatem/Projects/Simple3DViewer/game_content";
		shaders.push_back(ShaderDefinition(GL_VERTEX_SHADER, game_content + "/AmbientLightingDemo.vert"));
			shaders.push_back(ShaderDefinition(GL_FRAGMENT_SHADER, game_content + "/AmbientLightingDemo.frag"));
        mShaderProgram.BuildProgram(shaders);
        
        float size = 10.0f;
        float halfSize = size / 2.0f;
        
        // Create the vertex buffer
        VertexPositionTexture vertices[] =
        {
            VertexPositionTexture(vec4(-halfSize, 1.0f, 0.0, 1.0f), vec2(0.0f, 1.0f)),
            VertexPositionTexture(vec4(-halfSize, size + 1.0f, 0.0f, 1.0f), vec2(0.0f, 0.0f)),
            VertexPositionTexture(vec4(halfSize, size + 1.0f, 0.0f, 1.0f), vec2(1.0f, 0.0f)),
            VertexPositionTexture(vec4(halfSize, 1.0f, 0.0f, 1.0f), vec2(1.0f, 1.0f))
        };
        
        CreateVertexBuffer(vertices, sizeof(vertices)/sizeof(VertexPositionTexture), mVertexBuffer);
        
        // Create the index buffer
        unsigned int indices[] =
        {
            0, 2, 1,
            0, 3, 2
        };
        
        mIndexCount = sizeof(indices)/sizeof(unsigned int);
        CreateIndexBuffer(indices, mIndexCount, mIndexBuffer);
        
        // Find the WVP uniform location
        mWorldViewProjectionLocation = glGetUniformLocation(mShaderProgram.Program(), "WorldViewProjection");
        if (mWorldViewProjectionLocation == -1)
        {
            throw GameException("glGetUniformLocation() did not find uniform location.");
        }
        
        
        mAmbientColorLocation = glGetUniformLocation(mShaderProgram.Program(), "AmbientColor");
        if (mAmbientColorLocation == -1)
        {
            throw GameException("glGetUniformLocation() did not find uniform location.");
        }
        
        // Load the texture
        mColorTexture = SOIL_load_OGL_texture("EarthComposite.jpg", SOIL_LOAD_AUTO, SOIL_CREATE_NEW_ID, SOIL_FLAG_MIPMAPS | SOIL_FLAG_NTSC_SAFE_RGB);
        if (mColorTexture == 0)
        {
            throw GameException("SOIL_load_OGL_texture() failed.");
        }
        
//        int img_width1, img_height1;
//        unsigned char* img1 = SOIL_load_image("EarthComposite.jpg", &img_width1, &img_height1, NULL, 0);
//        glGenTextures(1, &mColorTexture);
//        glBindTexture(GL_TEXTURE_2D, mColorTexture);
//        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_width1, img_height1, 0, GL_RGB, GL_UNSIGNED_BYTE, img1);
        
        
        // Configure the texture samplers
        mTextureSamplers.resize(FilteringModeEnd);
        glGenSamplers(mTextureSamplers.size(), &mTextureSamplers[0]);
        
        for (FilteringMode mode = (FilteringMode)0; mode < FilteringModeEnd; mode = (FilteringMode)(mode + 1))
        {
            mTextureSamplersByFilteringMode[mode] = mTextureSamplers[mode];
        }
        
        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModePoint], GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModePoint], GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        
        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModeLinear], GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModeLinear], GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        
        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModePointMipMapPoint], GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModePointMipMapPoint], GL_TEXTURE_MAG_FILTER, GL_NEAREST_MIPMAP_NEAREST);
        
        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModeLinearMipMapPoint], GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModeLinearMipMapPoint], GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_NEAREST);
        
        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModePointMipMapLinear], GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR);
        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModePointMipMapLinear], GL_TEXTURE_MAG_FILTER, GL_NEAREST_MIPMAP_LINEAR);
        
        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModeTriLinear], GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glSamplerParameteri(mTextureSamplersByFilteringMode[FilteringModeTriLinear], GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        
        // Create the vertex array object
        glGenVertexArrays(1, &mVertexArrayObject);
        glBindVertexArray(mVertexArrayObject);
        
        glVertexAttribPointer(VertexAttributePosition, 4, GL_FLOAT, GL_FALSE, sizeof(VertexPositionTexture), (void*)offsetof(VertexPositionTexture, Position));
        glEnableVertexAttribArray(VertexAttributePosition);
        
        glVertexAttribPointer(VertexAttributeTextureCoordinate, 2, GL_FLOAT, GL_FALSE, sizeof(VertexPositionTexture), (void*)offsetof(VertexPositionTexture, TextureCoordinates));
        glEnableVertexAttribArray(VertexAttributeTextureCoordinate);
        
        glBindVertexArray(0);
        
        // Attach the keyboard handler
        using namespace std::placeholders;
        mKeyboardHandler = std::bind(&TexturedQuad::OnKey, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
        mGame->AddKeyboardHandler(mKeyboardHandler);
        
#if defined(DEBUG) || defined(_DEBUG)
        OutputFilteringMode();
#endif
        
                mAmbientLight = new Light(*mGame);
    }
    
    void TexturedQuad::Update(const GameTime& gameTime)
    {
        UpdateAmbientLight(gameTime);
    }
    
    void TexturedQuad::UpdateAmbientLight(const GameTime& gameTime)
    {
        static float ambientIntensity = 1.0f;
        //static float test_scalar = 0.10f;
        
        if (glfwGetKey(mGame->Window(), GLFW_KEY_L) && ambientIntensity < 1.0f)
        {
            ambientIntensity += (float)gameTime.ElapsedGameTime()/50.0f;
            ambientIntensity = min(ambientIntensity, 1.0f);
            
            mAmbientLight->SetColor(vec4((vec3)ambientIntensity, 1.0f));
        }
        
        if (glfwGetKey(mGame->Window(), GLFW_KEY_O) && ambientIntensity > 0.0f)
        {
            ambientIntensity -= (float)gameTime.ElapsedGameTime()/50.0f;
            ambientIntensity = max(ambientIntensity, 0.0f);
            
            mAmbientLight->SetColor(vec4((vec3)ambientIntensity, 1.0f));
        }
    }
    
    
    void TexturedQuad::Draw(const GameTime& gameTime)
    {
        glBindVertexArray(mVertexArrayObject);
        glBindBuffer(GL_ARRAY_BUFFER, mVertexBuffer);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndexBuffer);
        glBindTexture(GL_TEXTURE_2D, mColorTexture);
        
        glBindSampler(0, mTextureSamplersByFilteringMode[mActiveFilteringMode]);
        
        glUseProgram(mShaderProgram.Program());
        
        mat4 wvp = mCamera->ViewProjectionMatrix() * mWorldMatrix;
        glUniformMatrix4fv(mWorldViewProjectionLocation, 1, GL_FALSE, &wvp[0][0]);
        glUniform4fv(mAmbientColorLocation, 1, &mAmbientLight->Color()[0]);
        
        glEnable(GL_CULL_FACE);
        glFrontFace(GL_CCW);
        
        glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
    
    void TexturedQuad::CreateVertexBuffer(VertexPositionTexture* vertices, GLuint vertexCount, GLuint& vertexBuffer)
    {
        glGenBuffers(1, &vertexBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPositionTexture) * vertexCount, &vertices[0], GL_STATIC_DRAW);
    }
    
    void TexturedQuad::CreateIndexBuffer(unsigned int* indices, GLuint indexCount, GLuint& indexBuffer)
    {
        glGenBuffers(1, &indexBuffer);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * indexCount, indices, GL_STATIC_DRAW);
    }
    
    void TexturedQuad::OnKey(int key, int scancode, int action, int mods)
    {
        if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
        {
            FilteringMode activeMode = FilteringMode(mActiveFilteringMode + 1);
            if (activeMode >= FilteringModeEnd)
            {
                activeMode = (FilteringMode)(0);
            }
            
            mActiveFilteringMode = activeMode;
            
#if defined(DEBUG) || defined(_DEBUG)
            OutputFilteringMode();
#endif
        }
    }
    
    void TexturedQuad::OutputFilteringMode()
    {
        std::wostringstream message;
        message << "Active Filtering Mode: " << FilteringModeNames[mActiveFilteringMode].c_str() << "\n";
        //OutputDebugString(message.str().c_str());
    }
}