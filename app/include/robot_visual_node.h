#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/Material>
#include <osg/ref_ptr>

namespace sim_app {

struct LinkMeshInfo {
    std::string link_name;
    std::string mesh_path;
    std::vector<double> origin_xyz{0.0, 0.0, 0.0};
    std::vector<double> origin_rpy{0.0, 0.0, 0.0};
};

class RobotVisualNode : public osg::Group {
public:
    RobotVisualNode();
    virtual ~RobotVisualNode() = default;

    /**
     * @brief 构建机械臂 3D 视觉模型树
     * @param links 各连杆的名称与网格文件路径
     * @return 成功加载并构建返回 true
     */
    bool buildRobotModel(const std::vector<LinkMeshInfo>& links);

    /**
     * @brief 批量更新所有连杆的世界坐标位姿
     * @param link_names 连杆名称列表
     * @param link_poses 各连杆对应的世界坐标位姿 [x, y, z, qx, qy, qz, qw]
     */
    void updateLinkPoses(const std::vector<std::string>& link_names, 
                         const std::vector<std::vector<double>>& link_poses);

    /**
     * @brief 设置当前发生碰撞的连杆列表，并在 3D 场景中高亮红显
     * @param colliding_links 发生干涉冲突的连杆名称列表
     */
    void setCollisionHighlight(const std::vector<std::string>& colliding_links);

    /**
     * @brief 清除所有碰撞变红高亮，恢复常规材质
     */
    void clearCollisionHighlight();

    /**
     * @brief 设置末端工具坐标轴 (TCP RGB 坐标系) 的显示/隐藏
     */
    void showToolCoordinateFrame(bool show);

    /**
     * @brief 获取所有已加载的连杆名称列表
     */
    std::vector<std::string> getLinkNames() const;

private:
    struct LinkNode {
        osg::ref_ptr<osg::MatrixTransform> transform;
        osg::ref_ptr<osg::Node> visual_node;
        osg::ref_ptr<osg::StateSet> normal_stateset;
        osg::ref_ptr<osg::StateSet> collision_stateset;
        bool is_colliding{false};
    };

    std::unordered_map<std::string, LinkNode> link_nodes_;
    osg::ref_ptr<osg::MatrixTransform> tool_frame_transform_;
    bool show_tool_frame_{true};

    static osg::ref_ptr<osg::StateSet> createCollisionStateSet();
    static osg::ref_ptr<osg::Node> createCoordinateAxes(double length = 0.2, double radius = 0.008);
};

} // namespace sim_app
