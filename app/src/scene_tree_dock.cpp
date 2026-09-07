#include "scene_tree_dock.h"
#include "simulation_controller.h"
#include "osg_viewer_widget.h"
#include "add_obstacle_dialog.h"
#include "mesh_loader.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMenu>
#include <QMessageBox>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/Material>

namespace sim_app {

SceneTreeDock::SceneTreeDock(SimulationController* controller, OsgViewerWidget* viewer, QWidget* parent)
    : QDockWidget("场景树与障碍物 (Scene Hierarchy)", parent), controller_(controller), viewer_(viewer) {
    setObjectName("SceneTreeDock");
    setupUi();

    connect(controller_, &SimulationController::sigRobotLoaded, this, &SceneTreeDock::onRobotLoaded);
    connect(controller_, &SimulationController::sigObstaclesChanged, this, &SceneTreeDock::onObstaclesChanged);
    connect(controller_, &SimulationController::sigCollisionState, this, 
            [this](bool in_coll, const std::vector<std::string>& clinks, const auto&) {
                onCollisionState(in_coll, clinks);
            });

    if (controller_->isRobotLoaded()) {
        onRobotLoaded(true, QString::fromStdString(controller_->getRobotName()));
        onObstaclesChanged();
    }
}

void SceneTreeDock::setupUi() {
    QWidget* container = new QWidget(this);
    QVBoxLayout* layout = new QVBoxLayout(container);
    layout->setContentsMargins(6, 6, 6, 6);
    layout->setSpacing(6);

    tree_widget_ = new QTreeWidget(container);
    tree_widget_->setHeaderLabels({"节点名称 (Node)", "状态/类型 (Type/Status)"});
    tree_widget_->setColumnWidth(0, 160);
    tree_widget_->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(tree_widget_, &QTreeWidget::customContextMenuRequested, this, &SceneTreeDock::onCustomContextMenuRequested);

    item_robot_root_ = new QTreeWidgetItem(tree_widget_, {"🤖 机械臂连杆 (Robot Links)", ""});
    item_robot_root_->setExpanded(true);

    item_obstacles_root_ = new QTreeWidgetItem(tree_widget_, {"📦 场景障碍物 (Obstacles)", ""});
    item_obstacles_root_->setExpanded(true);

    layout->addWidget(tree_widget_, 1);

    QHBoxLayout* btnLayout = new QHBoxLayout();
    QPushButton* btnAdd = new QPushButton("➕ 添加障碍物...", container);
    btnAdd->setObjectName("btnPrimary");
    QPushButton* btnClear = new QPushButton("🗑️ 清空所有", container);
    btnClear->setObjectName("btnDanger");

    connect(btnAdd, &QPushButton::clicked, this, &SceneTreeDock::onAddObstacleClicked);
    connect(btnClear, &QPushButton::clicked, this, &SceneTreeDock::onClearObstaclesClicked);

    btnLayout->addWidget(btnAdd);
    btnLayout->addWidget(btnClear);
    layout->addLayout(btnLayout);

    setWidget(container);
}

void SceneTreeDock::onRobotLoaded(bool success, const QString& robot_name) {
    if (!success) return;
    item_robot_root_->setText(0, QString("🤖 机械臂: %1").arg(robot_name));
    
    // 清除既有子项
    qDeleteAll(item_robot_root_->takeChildren());

    const auto& links = controller_->getLinkNames();
    for (const auto& l : links) {
        new QTreeWidgetItem(item_robot_root_, {QString("🔗 %1").arg(QString::fromStdString(l)), "正常 (Normal)"});
    }
    item_robot_root_->setExpanded(true);
}

void SceneTreeDock::onObstaclesChanged() {
    qDeleteAll(item_obstacles_root_->takeChildren());

    const auto& obs = controller_->getObstacles();
    for (const auto& pair : obs) {
        const auto& d = pair.second;
        QString typeStr;
        switch (d.type) {
            case ObstacleType::BOX: typeStr = "🧊 Box 立方体"; break;
            case ObstacleType::SPHERE: typeStr = "⚪ Sphere 球体"; break;
            case ObstacleType::CYLINDER: typeStr = "🥫 Cylinder 圆柱体"; break;
            case ObstacleType::MESH: typeStr = "📐 Mesh 网格"; break;
            default: typeStr = "📦 障碍物"; break;
        }

        if (d.is_attached) {
            typeStr += QString(" [随动挂载: %1]").arg(QString::fromStdString(d.attached_link));
        }

        QTreeWidgetItem* item = new QTreeWidgetItem(item_obstacles_root_, {QString::fromStdString(d.name), typeStr});
        item->setData(0, Qt::UserRole, QString::fromStdString(d.name));
    }
    item_obstacles_root_->setExpanded(true);

    updateObstacleVisuals();
}

void SceneTreeDock::updateObstacleVisuals() {
    if (!viewer_) return;
    viewer_->clearObstaclesVisual();

    const auto& obs = controller_->getObstacles();
    for (const auto& pair : obs) {
        const auto& d = pair.second;
        osg::ref_ptr<osg::Node> node = nullptr;

        // 默认冷灰科技蓝半透明材质
        osg::Vec4 obsColor(0.25f, 0.55f, 0.85f, 0.9f);

        if (d.type == ObstacleType::BOX && d.dimensions.size() >= 3) {
            osg::ref_ptr<osg::Geode> geode = new osg::Geode();
            osg::ref_ptr<osg::Box> box = new osg::Box(osg::Vec3(0, 0, 0), d.dimensions[0], d.dimensions[1], d.dimensions[2]);
            osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(box.get());
            sd->setColor(obsColor);
            geode->addDrawable(sd.get());
            node = geode;
        } else if (d.type == ObstacleType::SPHERE && !d.dimensions.empty()) {
            osg::ref_ptr<osg::Geode> geode = new osg::Geode();
            osg::ref_ptr<osg::Sphere> sp = new osg::Sphere(osg::Vec3(0, 0, 0), d.dimensions[0]);
            osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(sp.get());
            sd->setColor(obsColor);
            geode->addDrawable(sd.get());
            node = geode;
        } else if (d.type == ObstacleType::CYLINDER && d.dimensions.size() >= 2) {
            osg::ref_ptr<osg::Geode> geode = new osg::Geode();
            osg::ref_ptr<osg::Cylinder> cyl = new osg::Cylinder(osg::Vec3(0, 0, 0), d.dimensions[0], d.dimensions[1]);
            osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(cyl.get());
            sd->setColor(obsColor);
            geode->addDrawable(sd.get());
            node = geode;
        } else if (d.type == ObstacleType::MESH && !d.file_path.empty()) {
            node = MeshLoader::loadMesh(d.file_path, obsColor);
        }

        if (node) {
            osg::Matrixd mat;
            if (d.pose.size() >= 7) {
                osg::Quat q(d.pose[3], d.pose[4], d.pose[5], d.pose[6]);
                mat.makeRotate(q);
                mat.postMultTranslate(osg::Vec3d(d.pose[0], d.pose[1], d.pose[2]));
            }
            viewer_->addObstacleVisual(d.name, node, mat);
        }
    }
}

void SceneTreeDock::onCollisionState(bool in_collision, const std::vector<std::string>& colliding_links) {
    (void)in_collision;
    // 更新机械臂连杆树节点的高亮状态
    for (int i = 0; i < item_robot_root_->childCount(); ++i) {
        QTreeWidgetItem* child = item_robot_root_->child(i);
        std::string linkName = child->text(0).toStdString();
        bool colliding = false;
        for (const auto& cl : colliding_links) {
            if (linkName == cl) {
                colliding = true;
                break;
            }
        }
        if (colliding) {
            child->setText(1, "干涉碰撞 (COLLISION)");
            child->setForeground(0, QColor("#f44336"));
            child->setForeground(1, QColor("#f44336"));
        } else {
            child->setText(1, "正常 (Normal)");
            child->setForeground(0, QColor("#e0e2ec"));
            child->setForeground(1, QColor("#9da3b4"));
        }
    }

    // 更新障碍物高亮状态
    const auto& obs = controller_->getObstacles();
    for (int i = 0; i < item_obstacles_root_->childCount(); ++i) {
        QTreeWidgetItem* child = item_obstacles_root_->child(i);
        std::string obsName = child->data(0, Qt::UserRole).toString().toStdString();
        bool colliding = false;
        for (const auto& cl : colliding_links) {
            if (obsName == cl) {
                colliding = true;
                break;
            }
        }
        if (viewer_) {
            viewer_->setObstacleCollisionHighlight(obsName, colliding);
        }
        if (colliding) {
            child->setForeground(0, QColor("#f44336"));
            child->setForeground(1, QColor("#f44336"));
        } else {
            child->setForeground(0, QColor("#e0e2ec"));
            child->setForeground(1, QColor("#9da3b4"));
        }
    }
}

void SceneTreeDock::onAddObstacleClicked() {
    AddObstacleDialog dlg(this);
    if (dlg.exec() == QDialog::Accepted) {
        dlg.applyToController(controller_);
    }
}

void SceneTreeDock::onClearObstaclesClicked() {
    if (QMessageBox::question(this, "清空确认", "确定要清空场景中所有用户添加的障碍物吗？") == QMessageBox::Yes) {
        controller_->clearAllObstacles();
    }
}

void SceneTreeDock::onCustomContextMenuRequested(const QPoint& pos) {
    QTreeWidgetItem* item = tree_widget_->itemAt(pos);
    if (!item || item->parent() != item_obstacles_root_) return;

    QString obsName = item->data(0, Qt::UserRole).toString();
    if (obsName.isEmpty()) return;

    QMenu menu(this);
    QAction* actAttach = menu.addAction("挂载到机械臂末端 (Attach to Tool0)");
    QAction* actDetach = menu.addAction("解除挂载 (Detach to World)");
    menu.addSeparator();
    QAction* actDelete = menu.addAction("删除此障碍物 (Delete)");

    QAction* selected = menu.exec(tree_widget_->viewport()->mapToGlobal(pos));
    if (selected == actAttach) {
        controller_->attachObstacle(obsName.toStdString(), "tool0");
    } else if (selected == actDetach) {
        controller_->detachObstacle(obsName.toStdString());
    } else if (selected == actDelete) {
        controller_->removeObstacle(obsName.toStdString());
    }
}

} // namespace sim_app
