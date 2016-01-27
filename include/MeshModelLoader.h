#pragma once
#include "DrawableGameComponent.h"
#include "ShaderProgram.h"
#include "VertexDeclarations.h"
#include "Game.h"

namespace Library
{
	class Mesh;
	class Light;
	//    class TexturedOBJMeshloader;
}

using namespace Library;

namespace Rendering
{
	class MeshModelLoader : public DrawableGameComponent
	{
		// RTTI_DECLARATIONS(FilteringModesDemo, DrawableGameComponent)

	public:
		MeshModelLoader(Game& game, Camera& camera);
		MeshModelLoader();

		virtual void Initialize() override;
		virtual void Draw(const GameTime& gameTime) override;
		virtual void Update(const GameTime& gameTime) override;

	private:
		//enum VertexAttribute
		//{
		//	VertexAttributePosition = 0,
		//	VertexAttributeTextureCoordinate = 1
		//};

		enum VertexAttribute
		{
			VertexAttributePosition = 0,
			VertexAttributeColor = 1
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

		~MeshModelLoader();
		MeshModelLoader(const MeshModelLoader& rhs);
		MeshModelLoader& operator=(const MeshModelLoader& rhs);

		//void CreateVertexBuffer(VertexPositionTexture* vertices, GLuint vertexCount, GLuint& vertexBuffer);
		void CreateVertexBuffer(const std::vector<VertexPositionTexture> &vertices, const std::vector<unsigned int> &nr_ver_per_img, std::vector<GLuint>& vertexBuffer);
		//void CreateIndexBuffer(unsigned int* indices, GLuint indexCount, GLuint& indexBuffer);
		void CreateIndexBuffer(const std::vector<unsigned int> &indices, const std::vector<unsigned int> &nr_ver_per_img, std::vector<GLuint>& indexBuffer);





		void CreateVertexBuffer(const Mesh& mesh, GLuint& tmpvertexBuffer) const;
		void CreateVertexBuffer(VertexPositionTexture* vertices, GLuint vertexCount, GLuint& tmpvertexBuffer) const;

		unsigned int VertexSize() const;




		//std::vector<TexturedOBJMeshloader*> mOBJMeshModel;




		void OnKey(int key, int scancode, int action, int mods);
		void OutputFilteringMode();

		void UpdateAmbientLight(const GameTime& gameTime);

		std::vector<unsigned int> m_img_nr;
		ShaderProgram mShaderProgram;
		GLuint mVertexArrayObject;
		GLuint mVertexBuffer;
		GLuint mIndexBuffer;
		GLint mWorldViewProjectionLocation;
		GLint mAmbientColorLocation;
		glm::mat4 mWorldMatrix;
		GLuint mIndexCount;
		std::vector<GLuint> mColorTexture;
		std::vector<GLuint> mTextureSamplers;
		std::map<FilteringMode, GLuint> mTextureSamplersByFilteringMode;
		FilteringMode mActiveFilteringMode;
		Game::KeyboardHandler mKeyboardHandler;
		Light* mAmbientLight;


		GLenum mAlternative_mode;
		GLenum mActive_mode;
	};
}

