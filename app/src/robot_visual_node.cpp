#include "robot_visual_node.h"
#include "mesh_loader.h"

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <iostream>

namespace sim_app {

RobotVisualNode::RobotVisualNode() {
    tool_frame_transform_ = new osg::MatrixTransform();
    tool_frame_transform_->addChild(createCoordinateAxes(0.25, 0.008));
    addChild(tool_frame_transform_.get());
}

bool RobotVisualNode::buildRobotModel(const std::vector<LinkMeshInfo>& links) {
    // 清除既有节点
    for (auto& pair : link_nodes_) {
        removeChild(pair.second.transform.get());
    }
    link_nodes_.clear();

    for (const auto& info : links) {
        LinkNode lnode;
        lnode.transform = new osg::MatrixTransform();
        
        // 机械臂标准工业浅灰/微黄涂装
        osg::Vec4 defaultColor(0.85f, 0.85f, 0.88f, 1.0f);
        if (info.link_name.find("base") != std::string::npos) {
            defaultColor.set(0.35f, 0.36f, 0.40f, 1.0f);
        } else if (info.link_name.find("J1") != std::string::npos || info.link_name.find("J3") != std::string::npos) {
            defaultColor.set(0.95f, 0.82f, 0.15f, 1.0f); // Fanuc 标志性醒目工业黄
        } else if (info.link_name.find("J2") != std::string::npos || info.link_name.find("J4") != std::string::npos) {
            defaultColor.set(0.95f, 0.82f, 0.15f, 1.0f);
        } else if (info.link_name.find("J5") != std::string::npos || info.link_name.find("J6") != std::string::npos) {
            defaultColor.set(0.3f, 0.3f, 0.35f, 1.0f);
        }

        if (!info.mesh_path.empty()) {
            lnode.visual_node = MeshLoader::loadMesh(info.mesh_path, defaultColor);
            if (lnode.visual_node) {
                lnode.transform->addChild(lnode.visual_node.get());
                lnode.normal_stateset = lnode.visual_node->getOrCreateStateSet();
            }
        }

        lnode.collision_stateset = createCollisionStateSet();
        link_nodes_[info.link_name] = lnode;
        addChild(lnode.transform.get());
    }

    return !link_nodes_.empty();
}

void RobotVisualNode::updateLinkPoses(const std::vector<std::string>& link_names, 
                                     const std::vector<std::vector<double>>& link_poses) {
    for (size_t i = 0; i < link_names.size() && i < link_poses.size(); ++i) {
        const std::string& name = link_names[i];
        const auto& pose = link_poses[i];
        if (pose.size() < 7) continue;

        auto it = link_nodes_.find(name);
        if (it != link_nodes_.end() && it->second.transform) {
            osg::Matrixd mat;
            double q_norm2 = pose[3]*pose[3] + pose[4]*pose[4] + pose[5]*pose[5] + pose[6]*pose[6];
            if (q_norm2 > 1e-4) {
                osg::Quat q(pose[3], pose[4], pose[5], pose[6]); // x, y, z, w
                mat.makeRotate(q);
            }
            mat.postMultTranslate(osg::Vec3d(pose[0], pose[1], pose[2]));
            it->second.transform->setMatrix(mat);

            if (name == "tool0" || name.find("tool") != std::string::npos) {
                tool_frame_transform_->setMatrix(mat);
            }
        }
    }
}

void RobotVisualNode::setCollisionHighlight(const std::vector<std::string>& colliding_links) {
    for (auto& pair : link_nodes_) {
        LinkNode& node = pair.second;
        bool should_collide = false;
        for (const auto& clink : colliding_links) {
            if (pair.first == clink) {
                should_collide = true;
                break;
            }
        }

        if (should_collide && !node.is_colliding) {
            node.is_colliding = true;
            if (node.visual_node) {
                node.visual_node->setStateSet(node.collision_stateset.get());
            }
        } else if (!should_collide && node.is_colliding) {
            node.is_colliding = false;
            if (node.visual_node) {
                node.visual_node->setStateSet(node.normal_stateset.get());
            }
        }
    }
}

void RobotVisualNode::clearCollisionHighlight() {
    for (auto& pair : link_nodes_) {
        LinkNode& node = pair.second;
        if (node.is_colliding) {
            node.is_colliding = false;
            if (node.visual_node) {
                node.visual_node->setStateSet(node.normal_stateset.get());
            }
        }
    }
}

void RobotVisualNode::showToolCoordinateFrame(bool show) {
    show_tool_frame_ = show;
    tool_frame_transform_->setNodeMask(show ? 0xFFFFFFFF : 0x0);
}

std::vector<std::string> RobotVisualNode::getLinkNames() const {
    std::vector<std::string> names;
    for (const auto& pair : link_nodes_) {
        names.push_back(pair.first);
    }
    return names;
}

osg::ref_ptr<osg::StateSet> RobotVisualNode::createCollisionStateSet() {
    osg::ref_ptr<osg::StateSet> ss = new osg::StateSet();
    osg::ref_ptr<osg::Material> redMat = new osg::Material();
    redMat->setColorMode(osg::Material::OFF);
    redMat->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(1.0f, 0.15f, 0.15f, 1.0f));
    redMat->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(0.6f, 0.05f, 0.05f, 1.0f));
    redMat->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0.8f, 0.2f, 0.2f, 1.0f));
    redMat->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(0.5f, 0.0f, 0.0f, 1.0f));
    redMat->setShininess(osg::Material::FRONT_AND_BACK, 64.0f);
    ss->setAttributeAndModes(redMat.get(), osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
    return ss;
}

osg::ref_ptr<osg::Node> RobotVisualNode::createCoordinateAxes(double length, double radius) {
    (void)radius;
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();

    osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array(6);
    // X axis (Red)
    (*verts)[0].set(0.0, 0.0, 0.0);
    (*verts)[1].set(length, 0.0, 0.0);
    // Y axis (Green)
    (*verts)[2].set(0.0, 0.0, 0.0);
    (*verts)[3].set(0.0, length, 0.0);
    // Z axis (Blue)
    (*verts)[4].set(0.0, 0.0, 0.0);
    (*verts)[5].set(0.0, 0.0, length);
    geom->setVertexArray(verts.get());

    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array(6);
    (*colors)[0].set(1.0f, 0.2f, 0.2f, 1.0f);
    (*colors)[1].set(1.0f, 0.2f, 0.2f, 1.0f);
    (*colors)[2].set(0.2f, 0.9f, 0.2f, 1.0f);
    (*colors)[3].set(0.2f, 0.9f, 0.2f, 1.0f);
    (*colors)[4].set(0.2f, 0.4f, 1.0f, 1.0f);
    (*colors)[5].set(0.2f, 0.4f, 1.0f, 1.0f);
    geom->setColorArray(colors.get(), osg::Array::BIND_PER_VERTEX);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 6));

    osg::ref_ptr<osg::StateSet> ss = geode->getOrCreateStateSet();
    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    osg::ref_ptr<osg::LineWidth> lw = new osg::LineWidth(3.5f);
    ss->setAttributeAndModes(lw.get(), osg::StateAttribute::ON);

    geode->addDrawable(geom.get());
    return geode;
}

} // namespace sim_app
