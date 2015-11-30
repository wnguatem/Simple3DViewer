#include "Quad.h"
#include "Game.h"
#include "GameException.h"
#include "ColorHelper.h"
#include "Camera.h"
#include "Utility.h"
#include "ShaderProgram.h"
#include "VertexDeclarations.h"
#include "VectorHelper.h"
#include "Model.h"
#include "Mesh.h"


using namespace glm;

namespace Rendering
{
    //RTTI_DEFINITIONS(Quad)
    
    Quad::Quad(Game& game, Camera& camera)
    : DrawableGameComponent(game, camera), mShaderProgram(), mVertexArrayObject(0), mVertexBuffer(0),
    mIndexBuffer(0), mWorldViewProjectionLocation(-1), mWorldMatrix(), mIndexCount(), m_input_data(game.getInputData())
    {
    }
    
    Quad::Quad(Game& game, Camera& camera, QuadType &quad, glm::vec4 &color)
    : DrawableGameComponent(game, camera), mShaderProgram(), mVertexArrayObject(0), mVertexBuffer(0),
    mIndexBuffer(0), mWorldViewProjectionLocation(-1), mWorldMatrix(), mIndexCount(), m_color(color), m_quad(quad), m_input_data(game.getInputData())
    {
    }
    
    Quad::Quad(Game& game, Camera& camera, const pcl::PointCloud<pcl::PointXYZ>::Ptr  &cloud, const glm::vec4 &color)
    : DrawableGameComponent(game, camera), mShaderProgram(), mVertexArrayObject(0), mVertexBuffer(0),
    mIndexBuffer(0), mWorldViewProjectionLocation(-1), mWorldMatrix(), mIndexCount(), m_color(color), m_cloud(cloud), m_input_data(game.getInputData())
    {
    }
    
    Quad::~Quad()
    {
        glDeleteBuffers(1, &mIndexBuffer);
        glDeleteBuffers(1, &mVertexBuffer);
        glDeleteVertexArrays(1, &mVertexArrayObject);
    }
    
    void Quad::Initialize()
    {
        //SetCurrentDirectory(Utility::ExecutableDirectory().c_str());
        
        // Build the shader program
        std::vector<ShaderDefinition> shaders;
        shaders.push_back(ShaderDefinition(GL_VERTEX_SHADER, "ModelDemo.vert"));
        shaders.push_back(ShaderDefinition(GL_FRAGMENT_SHADER, "ModelDemo.frag"));
        mShaderProgram.BuildProgram(shaders);
        
        // Load the model
        //std::unique_ptr<Model> model(new Model(*mGame, "Sphere.obj"));
        //std::unique_ptr<Model> model(new Model(*mGame, "C:\\williamnguatem\\Projects\\3DViewer\\build\\model_cloud.ply"));
        //std::unique_ptr<Model> model(new Model(*mGame, "C:\\williamnguatem\\Projects\\3DViewer\\build\\bunny\\bunny\\reconstruction\\bun_zipper.ply"));
        
        //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        
        //std::string filename = m_input_data;
        
        //pcl::io::loadPCDFile(m_input_data, *cloud);
        
        // Create the vertex and index buffers
        //Mesh* mesh = model->Meshes().at(0);
        //CreateVertexBuffer(*mesh, mVertexBuffer);
        //mesh->CreateIndexBuffer(mIndexBuffer);
        //mIndexCount = mesh->Indices().size();
        
        //create indices
        mIndexCount = 6;
        std::vector<unsigned int> cloud_indices(mIndexCount);
        // first triangle (bottom left - top left - top right)
        cloud_indices[0]=0;
        cloud_indices[1]=1;
        cloud_indices[2]=2;
        // second triangle (bottom left - top right - bottom right)
        cloud_indices[3]=0;
        cloud_indices[4]=2;
        cloud_indices[5]=3;
      //  GLubyte indices[] = {0,1,2,
      //      0,2,3};
        
        
//        GLfloat vertices[] = {-1, -1, 0, // bottom left corner
//            -1,  1, 0, // top left corner
//            1,  1, 0, // top right corner
//            1, -1, 0}; // bottom right corner
        
        
        
        // fill "indices" as needed
        GLuint indexBuffer;
        glGenBuffers(1, &indexBuffer);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * cloud_indices.size(), &cloud_indices[0], GL_STATIC_DRAW);
        mIndexBuffer = indexBuffer;
        mIndexCount = cloud_indices.size();
        
        //CreateVertexBuffer(*cloud, mVertexBuffer);
        CreateVertexBuffer(mVertexBuffer);
        //mesh->CreateIndexBuffer(mIndexBuffer);
        //mIndexCount = mesh->Indices().size();
        
        
        glGenVertexArrays(1, &mVertexArrayObject);
        glBindVertexArray(mVertexArrayObject);
        
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
    }
    
    void Quad::Draw(const GameTime& gameTime)
    {
        glBindVertexArray(mVertexArrayObject);
        glBindBuffer(GL_ARRAY_BUFFER, mVertexBuffer);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndexBuffer);
        
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
//        glPointSize(3.0f);
      //  glLineWidth(3.0f);
        
        
       // glVertexPointer(3, GL_FLOAT, 0, vertices);
       // glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_BYTE, indices);
        
        
        //glDrawArrays(GL_TRIANGLES, 0, mIndexCount);
        glDrawElements(GL_TRIANGLE_STRIP, mIndexCount, GL_UNSIGNED_INT, (void*)0 );
    }
    
    
    void Quad::CreateVertexBuffer(const Mesh& mesh, GLuint& vertexBuffer)
    {
        const std::vector<vec3>& sourceVertices = mesh.Vertices();
        
        std::vector<VertexPositionColor> vertices;
        vertices.reserve(sourceVertices.size());
        if (mesh.VertexColors().size() > 0)
        {
            std::vector<vec4>* vertexColors = mesh.VertexColors().at(0);
            assert(vertexColors->size() == sourceVertices.size());
            
            for (unsigned int i = 0; i < sourceVertices.size(); i++)
            {
                vec3 position = sourceVertices.at(i);
                vec4 color = vertexColors->at(i);
                vertices.push_back(VertexPositionColor(vec4(position.x, position.y, position.z, 1.0f), color));
            }
        }
        else
        {
            for (unsigned int i = 0; i < sourceVertices.size(); i++)
            {
                vec3 position = sourceVertices.at(i);
                vec4 color = ColorHelper::RandomColor();
                vertices.push_back(VertexPositionColor(vec4(position.x, position.y, position.z, 1.0f), color));
            }
        }
        
        glGenBuffers(1, &vertexBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPositionColor) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
    }
    
    void Quad::CreateVertexBuffer(const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud, GLuint& vertexBuffer)
    {
        //const std::vector<vec3>& sourceVertices = mesh.Vertices();
        size_t nr_points = cloud.points.size();
        
        std::vector<VertexPositionColor> vertices;
        vertices.reserve(nr_points);
        if (nr_points > 0)
        {
            //std::vector<vec4>* vertexColors = cloud.pointsmesh.VertexColors().at(0);
            //assert(vertexColors->size() == sourceVertices.size());
            for (unsigned int i = 0; i < nr_points; i++)
            {
                vec3 position = vec3(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
                //vec4 color = vec4(cloud.points[i].r, cloud.points[i].g, cloud.points[i].b, cloud.points[i].a);
                vec4 color = ColorHelper::RandomColor();
                vertices.push_back(VertexPositionColor(vec4(position.x, position.y, position.z, 1.0f), color));
            }
        }
        else
        {
            for (unsigned int i = 0; i < nr_points; i++)
            {
                vec3 position = vec3(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
                vec4 color = ColorHelper::RandomColor();
                vertices.push_back(VertexPositionColor(vec4(position.x, position.y, position.z, 1.0f), color));
            }
        }
        
        glGenBuffers(1, &vertexBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPositionColor) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
    }
    
    void Quad::CreateVertexBuffer(GLuint& vertexBuffer)
    {
        //const std::vector<vec3>& sourceVertices = mesh.Vertices();
        size_t nr_points = 4;
        
        //note that we have to swap the vertices accordingly since they are in anticlockwise order and swaping allows gl_traingle_strip to create two triangles
        //std::iter_swap(m_cloud->points.begin()+1, m_cloud->points.begin()+3);
        std::vector<VertexPositionColor> vertices;
        vertices.reserve(nr_points);
        if (nr_points > 0)
        {
            //std::vector<vec4>* vertexColors = cloud.pointsmesh.VertexColors().at(0);
            //assert(vertexColors->size() == sourceVertices.size());
            for (unsigned int i = 0; i < nr_points; i++)
            {
                vec3 position = vec3(m_cloud->points[i].x, m_cloud->points[i].y, m_cloud->points[i].z);
                vec4 color = m_color;
                vertices.push_back(VertexPositionColor(vec4(position.x, position.y, position.z, 1.0f), color));
            }
        }
        else
        {
            for (unsigned int i = 0; i < nr_points; i++)
            {
                vec3 position = vec3(m_cloud->points[i].x, m_cloud->points[i].y, m_cloud->points[i].z);
                vec4 color = ColorHelper::RandomColor();
                vertices.push_back(VertexPositionColor(vec4(position.x, position.y, position.z, 1.0f), color));
            }
        }
        
        glGenBuffers(1, &vertexBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPositionColor) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
    }
}
