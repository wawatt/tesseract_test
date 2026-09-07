#pragma once

#include <osgQOpenGL/osgQOpenGLWidget>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osgGA/TrackballManipulator>
#include <unordered_map>
#include <string>
#include <vector>

#include "robot_visual_node.h"
#include "robot_planner/robot_planner.h"

namespace sim_app {

class OsgViewerWidget : public osgQOpenGLWidget {
    Q_OBJECT

public:
    explicit OsgViewerWidget(QWidget* parent = nullptr);
    virtual ~OsgViewerWidget() = default;

    RobotVisualNode* getRobotVisualNode() const { return robot_node_.get(); }

    /**
     * @brief 场景中添加/更新障碍物 3D 模型
     */
    void addObstacleVisual(const std::string& name, osg::ref_ptr<osg::Node> node, const osg::Matrixd& mat);
    void updateObstacleTransform(const std::string& name, const osg::Matrixd& mat);
    void removeObstacleVisual(const std::string& name);
    void clearObstaclesVisual();
    void setObstacleCollisionHighlight(const std::string& name, bool highlight);

    /**
     * @brief 绘制碰撞接触点与法向量箭头
     */
    void updateContactMarkers(const std::vector<robot_planner::ContactInfo>& contacts);
    void clearContactMarkers();

    /**
     * @brief 重置相机观察角度
     */
    void resetCameraView();

    /**
     * @brief 切换地表网格与世界坐标轴显示
     */
    void setGridVisible(bool visible);
    void setAxesVisible(bool visible);

public slots:
    void onInitialized();

protected:
    void resizeGL(int w, int h) override;

private:
    void initSceneGraph();
    osg::ref_ptr<osg::Node> createGroundGrid(double size = 10.0, double step = 0.5);
    osg::ref_ptr<osg::Node> createWorldAxes(double length = 1.0);

    osg::ref_ptr<osg::Group> root_group_;
    osg::ref_ptr<osg::Node> ground_grid_;
    osg::ref_ptr<osg::Node> world_axes_;
    osg::ref_ptr<RobotVisualNode> robot_node_;
    osg::ref_ptr<osg::Group> obstacles_group_;
    osg::ref_ptr<osg::Group> collision_overlay_group_;

    struct ObstacleVisualItem {
        osg::ref_ptr<osg::MatrixTransform> transform;
        osg::ref_ptr<osg::Node> content_node;
        osg::ref_ptr<osg::StateSet> normal_ss;
        osg::ref_ptr<osg::StateSet> collision_ss;
        bool is_highlighted{false};
    };
    std::unordered_map<std::string, ObstacleVisualItem> obstacles_map_;

    osg::ref_ptr<osgGA::TrackballManipulator> camera_manipulator_;
};

} // namespace sim_app
