//******************************************************************************
//\par		Simple3DViewer - Simple Intelligent 3D Object Viewer
//\file		ModelMaterial.h
//\author	William Nguatem
//\note		Copyright (C) 
//\note		Bundeswehr University Munich
//\note		Institute of Applied Computer Science
//\note		Chair of Photogrammetry and Remote Sensing
//\note		Neubiberg, Germany
//\since		2015/03/26 
//******************************************************************************/


#ifndef _Model_Material_H_
#define _Model_Material_H_

#include "Common.h"

struct aiMaterial;

namespace Library
{
    enum TextureType
    {
        TextureTypeDifffuse = 0,
        TextureTypeSpecularMap,
        TextureTypeAmbient,
        TextureTypeEmissive,
        TextureTypeHeightmap,
        TextureTypeNormalMap,
        TextureTypeSpecularPowerMap,
        TextureTypeDisplacementMap,
        TextureTypeLightMap,
        TextureTypeEnd
    };
    
    class Model;

    class ModelMaterial
    {
        friend class Model;
        friend class Mesh;

    public:
        ModelMaterial(Model& model);
        ~ModelMaterial();

        Model& GetModel();
        const std::string& Name() const;
        const std::map<TextureType, std::vector<std::string>*>& Textures() const;

    private:		
        static void InitializeTextureTypeMappings();
        static std::map<TextureType, unsigned int> sTextureTypeMappings;

        ModelMaterial(Model& model, aiMaterial* material);
        ModelMaterial(const ModelMaterial& rhs);
        ModelMaterial& operator=(const ModelMaterial& rhs);

        Model& mModel;
        std::string mName;
        std::map<TextureType, std::vector<std::string>*> mTextures;		
    };
}
#endif//_Model_Material_H_