#pragma once

#include <string>
#include <vector>
#include <osg/Node>
#include <osg/Vec4>
#include <osg/ref_ptr>

namespace sim_app {

class MeshLoader {
public:
    /**
     * @brief 加载 3D 外部网格模型文件 (.dae, .obj, .stl, .ply 等) 并转换为 OSG 场景节点
     * @param file_path 模型文件绝对或相对路径
     * @param default_color 默认漫反射颜色
     * @return 成功返回 osg::Node 智能指针，失败返回 nullptr
     */
    static osg::ref_ptr<osg::Node> loadMesh(const std::string& file_path, 
                                           const osg::Vec4& default_color = osg::Vec4(0.85f, 0.85f, 0.88f, 1.0f));

    /**
     * @brief 读取网格文件的原始顶点与三角面片数据，用于向 robot_planner 添加 Mesh 障碍物
     * @param file_path 模型文件路径
     * @param vertices_out 顶点扁平数组 [x1, y1, z1, x2, y2, z2, ...]
     * @param faces_out 三角面顶点索引扁平数组 [v1, v2, v3, ...]
     * @return 成功返回 true
     */
    static bool loadRawMeshData(const std::string& file_path, 
                                std::vector<double>& vertices_out, 
                                std::vector<int>& faces_out);
};

} // namespace sim_app
