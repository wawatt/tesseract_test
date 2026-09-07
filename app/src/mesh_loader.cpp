#include "mesh_loader.h"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/LightModel>
#include <osg/Group>
#include <iostream>
#include <algorithm>

namespace sim_app {

static void extractNodeMeshes(const aiScene* scene, 
                              const aiNode* node, 
                              const aiMatrix4x4& parent_transform, 
                              const osg::Vec4& default_color,
                              osg::Geode* geode,
                              osg::Vec3& bmin,
                              osg::Vec3& bmax) {
    if (!node) return;
    aiMatrix4x4 transform = parent_transform * node->mTransformation;

    for (unsigned int m = 0; m < node->mNumMeshes; ++m) {
        const aiMesh* mesh = scene->mMeshes[node->mMeshes[m]];
        if (!mesh || mesh->mNumVertices == 0) continue;

        osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(mesh->mNumVertices);

        for (unsigned int v = 0; v < mesh->mNumVertices; ++v) {
            aiVector3D pos = transform * mesh->mVertices[v];
            (*vertices)[v].set(pos.x, pos.y, pos.z);
            bmin.x() = std::min(bmin.x(), pos.x);
            bmin.y() = std::min(bmin.y(), pos.y);
            bmin.z() = std::min(bmin.z(), pos.z);
            bmax.x() = std::max(bmax.x(), pos.x);
            bmax.y() = std::max(bmax.y(), pos.y);
            bmax.z() = std::max(bmax.z(), pos.z);
        }
        geom->setVertexArray(vertices.get());

        // 法向量
        if (mesh->HasNormals()) {
            aiMatrix3x3 norm_mat(transform);
            norm_mat.Inverse().Transpose();
            osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array(mesh->mNumVertices);
            for (unsigned int v = 0; v < mesh->mNumVertices; ++v) {
                aiVector3D norm = norm_mat * mesh->mNormals[v];
                norm.Normalize();
                (*normals)[v].set(norm.x, norm.y, norm.z);
            }
            geom->setNormalArray(normals.get(), osg::Array::BIND_PER_VERTEX);
        }

        // 面片索引
        osg::ref_ptr<osg::DrawElementsUInt> elements = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES);
        for (unsigned int f = 0; f < mesh->mNumFaces; ++f) {
            const aiFace& face = mesh->mFaces[f];
            if (face.mNumIndices == 3) {
                elements->push_back(face.mIndices[0]);
                elements->push_back(face.mIndices[1]);
                elements->push_back(face.mIndices[2]);
            }
        }
        if (!elements->empty()) {
            geom->addPrimitiveSet(elements.get());
        }

        // 漫反射色彩与材质
        osg::Vec4 diffuseColor = default_color;
        if (scene->HasMaterials() && mesh->mMaterialIndex < scene->mNumMaterials) {
            const aiMaterial* mat = scene->mMaterials[mesh->mMaterialIndex];
            aiColor4D color;
            if (mat->Get(AI_MATKEY_COLOR_DIFFUSE, color) == AI_SUCCESS) {
                if (color.r > 0.08f || color.g > 0.08f || color.b > 0.08f) {
                    diffuseColor.set(color.r, color.g, color.b, 1.0f);
                }
            }
        }

        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
        colors->push_back(diffuseColor);
        geom->setColorArray(colors.get(), osg::Array::BIND_OVERALL);

        osg::ref_ptr<osg::Material> material = new osg::Material();
        material->setColorMode(osg::Material::AMBIENT_AND_DIFFUSE);
        material->setDiffuse(osg::Material::FRONT_AND_BACK, diffuseColor);
        material->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(diffuseColor.r() * 0.45f, diffuseColor.g() * 0.45f, diffuseColor.b() * 0.45f, 1.0f));
        material->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0.3f, 0.3f, 0.3f, 1.0f));
        material->setShininess(osg::Material::FRONT_AND_BACK, 32.0f);

        osg::StateSet* ss = geom->getOrCreateStateSet();
        ss->setAttributeAndModes(material.get(), osg::StateAttribute::ON);
        ss->setMode(GL_LIGHTING, osg::StateAttribute::ON);
        ss->setMode(GL_NORMALIZE, osg::StateAttribute::ON);

        osg::ref_ptr<osg::LightModel> lm = new osg::LightModel();
        lm->setTwoSided(true);
        ss->setAttributeAndModes(lm.get(), osg::StateAttribute::ON);

        geode->addDrawable(geom.get());
    }

    for (unsigned int c = 0; c < node->mNumChildren; ++c) {
        extractNodeMeshes(scene, node->mChildren[c], transform, default_color, geode, bmin, bmax);
    }
}

osg::ref_ptr<osg::Node> MeshLoader::loadMesh(const std::string& file_path, const osg::Vec4& default_color) {
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(file_path,
        aiProcess_Triangulate |
        aiProcess_GenSmoothNormals |
        aiProcess_JoinIdenticalVertices |
        aiProcess_SortByPType);

    if (!scene || !scene->mRootNode || scene->mNumMeshes == 0) {
        std::cerr << "[MeshLoader] Error: Failed to load mesh from " << file_path 
                  << " : " << importer.GetErrorString() << std::endl;
        return nullptr;
    }

    // 与 ROS RViz 及 Tesseract mesh_parser 官方实现严格保持一致：
    // Assimp 默认执行 Y_UP 惯例，在遇到 Z_UP 的 COLLADA 文件时会自动在 mRootNode 注入 -90 度旋转。
    // 机器人 URDF 与 OSG 均遵循 Z_UP 惯例，必须重置 mRootNode 变换为单位阵，以防止轴向混乱。
    scene->mRootNode->mTransformation = aiMatrix4x4();

    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    osg::Vec3 bmin(1e9, 1e9, 1e9), bmax(-1e9, -1e9, -1e9);

    extractNodeMeshes(scene, scene->mRootNode, aiMatrix4x4(), default_color, geode.get(), bmin, bmax);

    std::cout << "[MeshLoader] Successfully imported: " << file_path << std::endl;
    std::cout << "[MeshLoader]   Bounds: [" << bmin.x() << ", " << bmin.y() << ", " << bmin.z() 
              << "] to [" << bmax.x() << ", " << bmax.y() << ", " << bmax.z() << "]" << std::endl;

    return geode;
}

static void extractRawNodeData(const aiScene* scene, 
                               const aiNode* node, 
                               const aiMatrix4x4& parent_transform, 
                               std::vector<double>& vertices_out, 
                               std::vector<int>& faces_out, 
                               int& vertex_offset) {
    if (!node) return;
    aiMatrix4x4 transform = parent_transform * node->mTransformation;

    for (unsigned int m = 0; m < node->mNumMeshes; ++m) {
        const aiMesh* mesh = scene->mMeshes[node->mMeshes[m]];
        if (!mesh) continue;

        for (unsigned int v = 0; v < mesh->mNumVertices; ++v) {
            aiVector3D pos = transform * mesh->mVertices[v];
            vertices_out.push_back(static_cast<double>(pos.x));
            vertices_out.push_back(static_cast<double>(pos.y));
            vertices_out.push_back(static_cast<double>(pos.z));
        }

        for (unsigned int f = 0; f < mesh->mNumFaces; ++f) {
            const aiFace& face = mesh->mFaces[f];
            if (face.mNumIndices == 3) {
                faces_out.push_back(vertex_offset + static_cast<int>(face.mIndices[0]));
                faces_out.push_back(vertex_offset + static_cast<int>(face.mIndices[1]));
                faces_out.push_back(vertex_offset + static_cast<int>(face.mIndices[2]));
            }
        }
        vertex_offset += static_cast<int>(mesh->mNumVertices);
    }

    for (unsigned int c = 0; c < node->mNumChildren; ++c) {
        extractRawNodeData(scene, node->mChildren[c], transform, vertices_out, faces_out, vertex_offset);
    }
}

bool MeshLoader::loadRawMeshData(const std::string& file_path, 
                                std::vector<double>& vertices_out, 
                                std::vector<int>& faces_out) {
    vertices_out.clear();
    faces_out.clear();

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(file_path,
        aiProcess_Triangulate |
        aiProcess_JoinIdenticalVertices |
        aiProcess_SortByPType);

    if (!scene || !scene->mRootNode || scene->mNumMeshes == 0) {
        std::cerr << "[MeshLoader] Error: Failed to read mesh data from " << file_path << std::endl;
        return false;
    }

    scene->mRootNode->mTransformation = aiMatrix4x4();
    int vertex_offset = 0;
    extractRawNodeData(scene, scene->mRootNode, aiMatrix4x4(), vertices_out, faces_out, vertex_offset);

    return !vertices_out.empty() && !faces_out.empty();
}

} // namespace sim_app
