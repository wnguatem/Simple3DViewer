#pragma once

#include "DrawableGameComponent.h"
#include "ShaderProgram.h"
#include "VertexDeclarations.h"
#include "Game.h"
#include "Mesh.h"

struct Face1
{
    float x[3], y[3], z[3];
    float u[3], v[3]; // UV co-ordinates, normalised to [0..1]
    int image_num; // best image assignment
};

namespace Library
{
    class Mesh;
    class Light;
}

using namespace Library;

namespace Rendering
{
    class TexturedOBJMeshloader : public DrawableGameComponent
    {
        // RTTI_DECLARATIONS(FilteringModesDemo, DrawableGameComponent)
        
    public:
        TexturedOBJMeshloader(Game& game, Camera& camera);
        //TexturedOBJMeshloader(Game& game, Camera& camera, const Mesh &meshData, const std::string &texture_filename);
        TexturedOBJMeshloader();
        
        virtual void Initialize() override;
        virtual void Draw(const GameTime& gameTime) override;
        virtual void Update(const GameTime& gameTime) override;
        
        void setTextureFile(const std::string &filename)
        {
            mTextureFilename = filename;
        };
        void setMesh(const Mesh &meshdata)
        {
            mMesh = meshdata;
        }
        
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
        
        ~TexturedOBJMeshloader();
        TexturedOBJMeshloader(const TexturedOBJMeshloader& rhs);
        TexturedOBJMeshloader& operator=(const TexturedOBJMeshloader& rhs);
        
        void ReOrderVetexBuffer(const std::vector<Face1> &faces, const std::vector<unsigned int> &img_nr, std::vector<unsigned int> &nr_p_img, std::vector<VertexPositionTexture> &vertices, std::vector<unsigned int> &ids);
        
        //void CreateVertexBuffer(VertexPositionTexture* vertices, GLuint vertexCount, GLuint& vertexBuffer);
        void CreateVertexBuffer(const std::vector<VertexPositionTexture> &vertices, const std::vector<unsigned int> &nr_ver_per_img, std::vector<GLuint>& vertexBuffer);
        //void CreateIndexBuffer(unsigned int* indices, GLuint indexCount, GLuint& indexBuffer);
        void CreateIndexBuffer(const std::vector<unsigned int> &indices, const std::vector<unsigned int> &nr_ver_per_img, std::vector<GLuint>& indexBuffer);
        
        
        
        
        
        void CreateVertexBuffer(const Mesh& mesh, GLuint& tmpvertexBuffer) const;
        void CreateVertexBuffer(VertexPositionTexture* vertices, GLuint vertexCount, GLuint& tmpvertexBuffer) const;
        
        unsigned int VertexSize() const;
        
        
        
        
        
        
        
        std::string mTextureFilename;
        Mesh mMesh;
        
        void OnKey(int key, int scancode, int action, int mods);
        void OutputFilteringMode();
        
        void UpdateAmbientLight(const GameTime& gameTime);
        
        std::vector<unsigned int> m_img_nr;
        ShaderProgram mShaderProgram;
        std::vector<GLuint> mVertexArrayObject;
        std::vector<GLuint> mVertexBuffer;
        std::vector <Face1> m_faces;
        std::vector<GLuint> mIndexBuffer;
        GLint mWorldViewProjectionLocation;
        GLint mAmbientColorLocation;
        glm::mat4 mWorldMatrix;
        std::vector<GLuint> mIndexCount;
        std::vector<GLuint> mColorTexture;
        std::vector<GLuint> mTextureSamplers;
        std::map<FilteringMode, GLuint> mTextureSamplersByFilteringMode;
        FilteringMode mActiveFilteringMode;
        Game::KeyboardHandler mKeyboardHandler;
        Light* mAmbientLight;
    };
}

