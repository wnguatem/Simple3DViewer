//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		ProxyModel.h
//\author	William Nguatem
//\note		Copyright (C) 
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26 
//******************************************************************************/


#ifndef _Proxy_Model_H_
#define _Proxy_Model_H_

#include "Common.h"
#include "DrawableGameComponent.h"
#include "BasicEffect.h"

namespace Library
{
	class Mesh;

	class ProxyModel : public DrawableGameComponent
	{
		//RTTI_DECLARATIONS(ProxyModel, DrawableGameComponent)

	public:
		ProxyModel(Game& game, Camera& camera, const std::string& modelFileName, float scale = 1.0f);
		~ProxyModel();

		const glm::vec3& Position() const;
        const glm::vec3& Direction() const;
        const glm::vec3& Up() const;
        const glm::vec3& Right() const;

		bool& DisplayWireframe();

		//void SetPosition(FLOAT x, FLOAT y, FLOAT z);
		void SetPosition(float x, float y, float z);
        void SetPosition(const glm::vec3& position);

        void ApplyRotation(const glm::mat4& transform);

		virtual void Initialize() override;
		virtual void Update(const GameTime& gameTime) override;		
		virtual void Draw(const GameTime& gameTime) override;

	private:
		ProxyModel();
		ProxyModel(const ProxyModel& rhs);
		ProxyModel& operator=(const ProxyModel& rhs);

		std::string mModelFileName;
		BasicEffect mShaderProgram;
		GLuint mVertexArrayObject;
		GLuint mVertexBuffer;
		GLuint mIndexBuffer;
		unsigned int mIndexCount;
        
		glm::mat4 mWorldMatrix;
		glm::mat4 mScaleMatrix;

		bool mDisplayWireframe;
		glm::vec3 mPosition;
		glm::vec3 mDirection;
        glm::vec3 mUp;
        glm::vec3 mRight;
	};
}
#endif//_Proxy_Model_H_