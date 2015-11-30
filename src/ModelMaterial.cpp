#include "ModelMaterial.h"
#include "GameException.h"
#include "Utility.h"
#include <assimp/scene.h>

namespace Library
{
    std::map<TextureType, unsigned int> ModelMaterial::sTextureTypeMappings;

    ModelMaterial::ModelMaterial(Model& model)
        : mModel(model), mTextures()
    {
        InitializeTextureTypeMappings();
    }

    ModelMaterial::ModelMaterial(Model& model, aiMaterial* material)
        : mModel(model), mTextures()
    {
        InitializeTextureTypeMappings();

        aiString name;
        material->Get(AI_MATKEY_NAME, name);
        mName = name.C_Str();

        for (TextureType textureType = (TextureType)0; textureType < TextureTypeEnd; textureType = (TextureType)(textureType + 1))
        {
            aiTextureType mappedTextureType = (aiTextureType)sTextureTypeMappings[textureType];

            unsigned int textureCount = material->GetTextureCount(mappedTextureType);
            if (textureCount > 0)
            {
                std::vector<std::string>* textures = new std::vector<std::string>();
                mTextures.insert(std::pair<TextureType, std::vector<std::string>*>(textureType, textures));

                textures->reserve(textureCount);
                for (unsigned int textureIndex = 0; textureIndex < textureCount; textureIndex++)
                {
                    aiString path;
                    if (material->GetTexture(mappedTextureType, textureIndex, &path) == AI_SUCCESS)
                    {
                        std::string wPath;
                        Utility::ToWideString(path.C_Str(), wPath);

                        textures->push_back(wPath);
                    }
                }
            }
        }
    }

    ModelMaterial::~ModelMaterial()
    {
        for (std::pair<TextureType, std::vector<std::string>*> textures : mTextures)
        {
            DeleteObject(textures.second);
        }
    }

    Model& ModelMaterial::GetModel()
    {
        return mModel;
    }

    const std::string& ModelMaterial::Name() const
    {
        return mName;
    }

    const std::map<TextureType, std::vector<std::string>*>& ModelMaterial::Textures() const
    {
        return mTextures;
    }

    void ModelMaterial::InitializeTextureTypeMappings()
    {
        if (sTextureTypeMappings.size() != TextureTypeEnd)
        {
            sTextureTypeMappings[TextureTypeDifffuse] = aiTextureType_DIFFUSE;
            sTextureTypeMappings[TextureTypeSpecularMap] = aiTextureType_SPECULAR;
            sTextureTypeMappings[TextureTypeAmbient] = aiTextureType_AMBIENT;
            sTextureTypeMappings[TextureTypeHeightmap] = aiTextureType_HEIGHT;
            sTextureTypeMappings[TextureTypeNormalMap] = aiTextureType_NORMALS;
            sTextureTypeMappings[TextureTypeSpecularPowerMap] = aiTextureType_SHININESS;
            sTextureTypeMappings[TextureTypeDisplacementMap] = aiTextureType_DISPLACEMENT;
            sTextureTypeMappings[TextureTypeLightMap] = aiTextureType_LIGHTMAP;
        }
    }
}