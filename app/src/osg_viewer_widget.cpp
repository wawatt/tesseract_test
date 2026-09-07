#include "osg_viewer_widget.h"

#include <osgViewer/Viewer>
#include <osg/Camera>
#include <osg/Viewport>
#include <osg/LightSource>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/Material>
#include <osgDB/ReadFile>

namespace sim_app {

OsgViewerWidget::OsgViewerWidget(QWidget* parent)
    : osgQOpenGLWidget(parent) {
    initSceneGraph();
    connect(this, &osgQOpenGLWidget::initialized, this, &OsgViewerWidget::onInitialized);
}

void OsgViewerWidget::onInitialized() {
    osgViewer::Viewer* viewer = getOsgViewer();
    if (!viewer) return;

    // 科技深灰工业背景底色
    osg::Camera* cam = viewer->getCamera();
    cam->setClearColor(osg::Vec4(0.12f, 0.13f, 0.16f, 1.0f));

    // 轨道相机控制器
    camera_manipulator_ = new osgGA::TrackballManipulator();
    camera_manipulator_->setAutoComputeHomePosition(false);
    camera_manipulator_->setHomePosition(
        osg::Vec3d(3.5, -3.8, 2.8),
        osg::Vec3d(0.0, 0.0, 0.9),
        osg::Vec3d(0.0, 0.0, 1.0)
    );
    viewer->setCameraManipulator(camera_manipulator_.get());

    viewer->setSceneData(root_group_.get());
    camera_manipulator_->home(0.0);

    // 同步初始视口与透视投影矩阵 (防止宽高比失真导致上下拉伸)
    qreal ratio = devicePixelRatio();
    int pw = static_cast<int>(width() * ratio);
    int ph = static_cast<int>(height() * ratio);
    if (pw > 0 && ph > 0) {
        cam->setViewport(new osg::Viewport(0, 0, pw, ph));
        double aspect = static_cast<double>(pw) / static_cast<double>(ph);
        cam->setProjectionMatrixAsPerspective(45.0, aspect, 0.1, 1000.0);
    }

    update();
}

void OsgViewerWidget::resizeGL(int w, int h) {
    osgQOpenGLWidget::resizeGL(w, h);
    osgViewer::Viewer* viewer = getOsgViewer();
    if (viewer) {
        if (viewer->getEventQueue()) {
            viewer->getEventQueue()->windowResize(0, 0, w, h);
        }
        osg::Camera* cam = viewer->getCamera();
        if (cam && h > 0) {
            cam->setViewport(new osg::Viewport(0, 0, w, h));
            double aspect = static_cast<double>(w) / static_cast<double>(h);
            cam->setProjectionMatrixAsPerspective(45.0, aspect, 0.1, 1000.0);
        }
    }
}

void OsgViewerWidget::initSceneGraph() {
    root_group_ = new osg::Group();
    root_group_->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);
    root_group_->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
    root_group_->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);

    // 工业柔和环境与主定向光源
    osg::ref_ptr<osg::LightSource> lightSource = new osg::LightSource();
    osg::ref_ptr<osg::Light> light = new osg::Light();
    light->setLightNum(0);
    light->setPosition(osg::Vec4(5.0f, -5.0f, 10.0f, 0.0f)); // 定向光
    light->setDiffuse(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    light->setAmbient(osg::Vec4(0.45f, 0.45f, 0.50f, 1.0f));
    light->setSpecular(osg::Vec4(0.7f, 0.7f, 0.7f, 1.0f));
    lightSource->setLight(light.get());
    lightSource->setStateSetModes(*root_group_->getOrCreateStateSet(), osg::StateAttribute::ON);
    root_group_->addChild(lightSource.get());

    // 辅助补光
    osg::ref_ptr<osg::LightSource> fillLightSource = new osg::LightSource();
    osg::ref_ptr<osg::Light> fillLight = new osg::Light();
    fillLight->setLightNum(1);
    fillLight->setPosition(osg::Vec4(-5.0f, 5.0f, 6.0f, 0.0f));
    fillLight->setDiffuse(osg::Vec4(0.6f, 0.62f, 0.68f, 1.0f));
    fillLight->setAmbient(osg::Vec4(0.30f, 0.30f, 0.35f, 1.0f));
    fillLightSource->setLight(fillLight.get());
    fillLightSource->setStateSetModes(*root_group_->getOrCreateStateSet(), osg::StateAttribute::ON);
    root_group_->addChild(fillLightSource.get());

    // 地表网格
    ground_grid_ = createGroundGrid(8.0, 0.5);
    root_group_->addChild(ground_grid_.get());

    // 世界坐标三轴
    world_axes_ = createWorldAxes(1.0);
    root_group_->addChild(world_axes_.get());

    // 机器人节点
    robot_node_ = new RobotVisualNode();
    root_group_->addChild(robot_node_.get());

    // 障碍物组
    obstacles_group_ = new osg::Group();
    root_group_->addChild(obstacles_group_.get());

    // 碰撞接触点法线覆盖层
    collision_overlay_group_ = new osg::Group();
    root_group_->addChild(collision_overlay_group_.get());
}

osg::ref_ptr<osg::Node> OsgViewerWidget::createGroundGrid(double size, double step) {
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();

    std::vector<osg::Vec3> lines;
    for (double x = -size; x <= size; x += step) {
        lines.emplace_back(x, -size, 0.0);
        lines.emplace_back(x, size, 0.0);
    }
    for (double y = -size; y <= size; y += step) {
        lines.emplace_back(-size, y, 0.0);
        lines.emplace_back(size, y, 0.0);
    }

    osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array(lines.size());
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array(lines.size());

    for (size_t i = 0; i < lines.size(); ++i) {
        (*verts)[i] = lines[i];
        bool is_major = (std::abs(lines[i].x()) < 1e-4 || std::abs(lines[i].y()) < 1e-4);
        if (is_major) {
            (*colors)[i] = osg::Vec4(0.25f, 0.45f, 0.65f, 0.9f); // 坐标轴深科技蓝
        } else {
            (*colors)[i] = osg::Vec4(0.18f, 0.21f, 0.26f, 0.6f); // 细格线
        }
    }

    geom->setVertexArray(verts.get());
    geom->setColorArray(colors.get(), osg::Array::BIND_PER_VERTEX);
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, verts->size()));

    osg::ref_ptr<osg::StateSet> ss = geode->getOrCreateStateSet();
    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    osg::ref_ptr<osg::LineWidth> lw = new osg::LineWidth(1.0f);
    ss->setAttributeAndModes(lw.get(), osg::StateAttribute::ON);

    geode->addDrawable(geom.get());
    return geode;
}

osg::ref_ptr<osg::Node> OsgViewerWidget::createWorldAxes(double length) {
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();

    osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array(6);
    (*verts)[0].set(0.0, 0.0, 0.0);
    (*verts)[1].set(length, 0.0, 0.0);
    (*verts)[2].set(0.0, 0.0, 0.0);
    (*verts)[3].set(0.0, length, 0.0);
    (*verts)[4].set(0.0, 0.0, 0.0);
    (*verts)[5].set(0.0, 0.0, length);
    geom->setVertexArray(verts.get());

    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array(6);
    (*colors)[0].set(0.9f, 0.2f, 0.2f, 1.0f); // X 轴 红
    (*colors)[1].set(0.9f, 0.2f, 0.2f, 1.0f);
    (*colors)[2].set(0.2f, 0.85f, 0.2f, 1.0f); // Y 轴 绿
    (*colors)[3].set(0.2f, 0.85f, 0.2f, 1.0f);
    (*colors)[4].set(0.2f, 0.4f, 0.95f, 1.0f); // Z 轴 蓝
    (*colors)[5].set(0.2f, 0.4f, 0.95f, 1.0f);
    geom->setColorArray(colors.get(), osg::Array::BIND_PER_VERTEX);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 6));

    osg::ref_ptr<osg::StateSet> ss = geode->getOrCreateStateSet();
    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    osg::ref_ptr<osg::LineWidth> lw = new osg::LineWidth(3.0f);
    ss->setAttributeAndModes(lw.get(), osg::StateAttribute::ON);

    geode->addDrawable(geom.get());
    return geode;
}

void OsgViewerWidget::addObstacleVisual(const std::string& name, osg::ref_ptr<osg::Node> node, const osg::Matrixd& mat) {
    removeObstacleVisual(name);

    ObstacleVisualItem item;
    item.transform = new osg::MatrixTransform();
    item.transform->setMatrix(mat);
    item.content_node = node;
    if (node) {
        item.transform->addChild(node.get());
        item.normal_ss = node->getOrCreateStateSet();
    }

    osg::ref_ptr<osg::StateSet> coll_ss = new osg::StateSet();
    osg::ref_ptr<osg::Material> redMat = new osg::Material();
    redMat->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(1.0f, 0.2f, 0.2f, 0.85f));
    redMat->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(0.6f, 0.1f, 0.1f, 1.0f));
    redMat->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(0.4f, 0.0f, 0.0f, 1.0f));
    coll_ss->setAttributeAndModes(redMat.get(), osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
    item.collision_ss = coll_ss;

    obstacles_map_[name] = item;
    obstacles_group_->addChild(item.transform.get());
    update();
}

void OsgViewerWidget::updateObstacleTransform(const std::string& name, const osg::Matrixd& mat) {
    auto it = obstacles_map_.find(name);
    if (it != obstacles_map_.end() && it->second.transform) {
        it->second.transform->setMatrix(mat);
        update();
    }
}

void OsgViewerWidget::removeObstacleVisual(const std::string& name) {
    auto it = obstacles_map_.find(name);
    if (it != obstacles_map_.end()) {
        obstacles_group_->removeChild(it->second.transform.get());
        obstacles_map_.erase(it);
        update();
    }
}

void OsgViewerWidget::clearObstaclesVisual() {
    obstacles_group_->removeChildren(0, obstacles_group_->getNumChildren());
    obstacles_map_.clear();
    update();
}

void OsgViewerWidget::setObstacleCollisionHighlight(const std::string& name, bool highlight) {
    auto it = obstacles_map_.find(name);
    if (it != obstacles_map_.end() && it->second.content_node) {
        if (highlight && !it->second.is_highlighted) {
            it->second.is_highlighted = true;
            it->second.content_node->setStateSet(it->second.collision_ss.get());
            update();
        } else if (!highlight && it->second.is_highlighted) {
            it->second.is_highlighted = false;
            it->second.content_node->setStateSet(it->second.normal_ss.get());
            update();
        }
    }
}

void OsgViewerWidget::updateContactMarkers(const std::vector<robot_planner::ContactInfo>& contacts) {
    clearContactMarkers();
    if (contacts.empty()) return;

    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    osg::ref_ptr<osg::Geometry> lineGeom = new osg::Geometry();
    osg::ref_ptr<osg::Geometry> ptsGeom = new osg::Geometry();

    std::vector<osg::Vec3> lines;
    std::vector<osg::Vec3> pts;

    for (const auto& c : contacts) {
        if (c.point1.size() >= 3 && c.point2.size() >= 3) {
            osg::Vec3 p1(c.point1[0], c.point1[1], c.point1[2]);
            osg::Vec3 p2(c.point2[0], c.point2[1], c.point2[2]);
            pts.push_back(p1);
            pts.push_back(p2);
            lines.push_back(p1);
            lines.push_back(p2);

            // 绘制法向量微箭头
            if (c.normal.size() >= 3) {
                osg::Vec3 n(c.normal[0], c.normal[1], c.normal[2]);
                lines.push_back(p2);
                lines.push_back(p2 + n * 0.12);
            }
        }
    }

    if (!pts.empty()) {
        osg::ref_ptr<osg::Vec3Array> ptsArray = new osg::Vec3Array(pts.size());
        osg::ref_ptr<osg::Vec4Array> ptsColors = new osg::Vec4Array(pts.size());
        for (size_t i = 0; i < pts.size(); ++i) {
            (*ptsArray)[i] = pts[i];
            (*ptsColors)[i] = osg::Vec4(1.0f, 0.9f, 0.0f, 1.0f); // 黄色接触点
        }
        ptsGeom->setVertexArray(ptsArray.get());
        ptsGeom->setColorArray(ptsColors.get(), osg::Array::BIND_PER_VERTEX);
        ptsGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, pts.size()));

        osg::ref_ptr<osg::Point> ptSize = new osg::Point(8.0f);
        ptsGeom->getOrCreateStateSet()->setAttributeAndModes(ptSize.get(), osg::StateAttribute::ON);
        ptsGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        geode->addDrawable(ptsGeom.get());
    }

    if (!lines.empty()) {
        osg::ref_ptr<osg::Vec3Array> linesArray = new osg::Vec3Array(lines.size());
        osg::ref_ptr<osg::Vec4Array> linesColors = new osg::Vec4Array(lines.size());
        for (size_t i = 0; i < lines.size(); ++i) {
            (*linesArray)[i] = lines[i];
            (*linesColors)[i] = osg::Vec4(1.0f, 0.2f, 0.2f, 1.0f); // 红色接触法线
        }
        lineGeom->setVertexArray(linesArray.get());
        lineGeom->setColorArray(linesColors.get(), osg::Array::BIND_PER_VERTEX);
        lineGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, lines.size()));

        osg::ref_ptr<osg::LineWidth> lw = new osg::LineWidth(3.0f);
        lineGeom->getOrCreateStateSet()->setAttributeAndModes(lw.get(), osg::StateAttribute::ON);
        lineGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        geode->addDrawable(lineGeom.get());
    }

    collision_overlay_group_->addChild(geode.get());
    update();
}

void OsgViewerWidget::clearContactMarkers() {
    collision_overlay_group_->removeChildren(0, collision_overlay_group_->getNumChildren());
    update();
}

void OsgViewerWidget::resetCameraView() {
    if (camera_manipulator_) {
        camera_manipulator_->setAutoComputeHomePosition(false);
        camera_manipulator_->setHomePosition(
            osg::Vec3d(3.5, -3.8, 2.8),
            osg::Vec3d(0.0, 0.0, 0.9),
            osg::Vec3d(0.0, 0.0, 1.0)
        );
        camera_manipulator_->home(0.0);
    }
    osgViewer::Viewer* viewer = getOsgViewer();
    if (viewer && viewer->getCamera()) {
        osg::Camera* cam = viewer->getCamera();
        qreal ratio = devicePixelRatio();
        int pw = static_cast<int>(width() * ratio);
        int ph = static_cast<int>(height() * ratio);
        if (pw > 0 && ph > 0) {
            cam->setViewport(new osg::Viewport(0, 0, pw, ph));
            double aspect = static_cast<double>(pw) / static_cast<double>(ph);
            cam->setProjectionMatrixAsPerspective(45.0, aspect, 0.1, 1000.0);
        }
    }
    update();
}

void OsgViewerWidget::setGridVisible(bool visible) {
    if (ground_grid_) {
        ground_grid_->setNodeMask(visible ? 0xFFFFFFFF : 0x0);
        update();
    }
}

void OsgViewerWidget::setAxesVisible(bool visible) {
    if (world_axes_) {
        world_axes_->setNodeMask(visible ? 0xFFFFFFFF : 0x0);
        update();
    }
}

} // namespace sim_app
