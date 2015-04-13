#pragma once

#include "DrawableGameComponent.h"
#include "ShaderProgram.h"
#include "VertexDeclarations.h"
#include "Game.h"

namespace Library
{
    class Mesh;
    class Light;
}

using namespace Library;

namespace Rendering
{
    class TexturedQuad : public DrawableGameComponent
    {
       // RTTI_DECLARATIONS(FilteringModesDemo, DrawableGameComponent)
        
    public:
        TexturedQuad(Game& game, Camera& camera);
        ~TexturedQuad();
        
        virtual void Initialize() override;
        virtual void Draw(const GameTime& gameTime) override;
        virtual void Update(const GameTime& gameTime) override;
        
    private:
        enum VertexAttribute
        {
            VertexAttributePosition = 0,
            VertexAttributeTextureCoordinate = 1
        };
        
        enum FilteringMode
        {
            FilteringModePoint = 0,
            FilteringModeLinear,
            FilteringModePointMipMapPoint,
            FilteringModeLinearMipMapPoint,
            FilteringModePointMipMapLinear,
            FilteringModeTriLinear,
            FilteringModeEnd
        };
        
        static const std::string FilteringModeNames[];
        
        TexturedQuad();
        TexturedQuad(const TexturedQuad& rhs);
        TexturedQuad& operator=(const TexturedQuad& rhs);
        
        void CreateVertexBuffer(VertexPositionTexture* vertices, GLuint vertexCount, GLuint& vertexBuffer);
        void CreateIndexBuffer(unsigned int* indices, GLuint indexCount, GLuint& indexBuffer);
        void OnKey(int key, int scancode, int action, int mods);
        void OutputFilteringMode();
        
        void UpdateAmbientLight(const GameTime& gameTime);
        
        ShaderProgram mShaderProgram;
        GLuint mVertexArrayObject;
        GLuint mVertexBuffer;
        GLuint mIndexBuffer;
        GLint mWorldViewProjectionLocation;
        GLint mAmbientColorLocation;
        glm::mat4 mWorldMatrix;
        GLuint mIndexCount;
        GLuint mColorTexture;
        std::vector<GLuint> mTextureSamplers;
        std::map<FilteringMode, GLuint> mTextureSamplersByFilteringMode;
        FilteringMode mActiveFilteringMode;
        Game::KeyboardHandler mKeyboardHandler;
        Light* mAmbientLight;
    };
}

