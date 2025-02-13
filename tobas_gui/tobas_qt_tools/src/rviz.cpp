#include <QDebug>
#include <OgreMaterialManager.h>
#include <rviz_common/yaml_config_reader.hpp>

#include "tobas_qt_tools/rviz.hpp"

namespace qt
{
RvizFrameManager::RvizFrameManager(const std::string& node_name)
{
  // Initialize ROS node
  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  // Create Rviz ROS interface
  node_ = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(node_name);
}

void RvizFrameManager::initialize(const QString& config_path, QWidget* parent)
{
  removeDefaultColorMaterials();

  // Read configuration
  rviz_common::YamlConfigReader reader;
  rviz_common::Config config;
  reader.readFile(config, config_path);

  // Setup visualization frame
  frame_ = new rviz_common::VisualizationFrame(node_, parent);
  try
  {
    frame_->initialize(node_);
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(rawNode()->get_logger(), e.what());
    throw;
  }
  frame_->setHelpPath("");
  frame_->setSplashPath("");
  frame_->load(config);
  frame_->setMenuBar(nullptr);
  frame_->setStatusBar(nullptr);
  frame_->setHideButtonVisibility(false);
  frame_->setStyleSheet("QSizeGrip { width: 0px; height: 0px; }");  // Remove sizegrip

  // Get child instances
  manager_ = frame_->getManager();
  display_group_ = manager_->getRootDisplayGroup();
}

rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr RvizFrameManager::rvizNode()
{
  return node_;
}

rclcpp::Node::SharedPtr RvizFrameManager::rawNode()
{
  if (node_ == nullptr)
    throw std::runtime_error("Rviz node is not initialized.");

  return node_->get_raw_node();
}

QWidget* RvizFrameManager::widget()
{
  return frame_;
}

void RvizFrameManager::resetTime()
{
  manager_->resetTime();
}

QString RvizFrameManager::getFixedFrame() const
{
  return manager_->getFixedFrame();
}

void RvizFrameManager::setFixedFrame(const QString& frame)
{
  manager_->setFixedFrame(frame);
}

rviz_common::Display* RvizFrameManager::getDisplay(const QString& name)
{
  for (int i = 0; i < display_group_->numDisplays(); ++i)
  {
    const auto display = display_group_->getDisplayAt(i);

    if (display == nullptr)
    {
      qWarning() << "Failed to get display of index " << QString::number(i);
      continue;
    };

    if (display->getName() == name)
      return display;
  }

  qWarning() << "Failed to find display named \"" << name << "\"";
  return nullptr;
}

void RvizFrameManager::removeDefaultColorMaterials()
{
  const auto material_manager = Ogre::MaterialManager::getSingletonPtr();

  if (material_manager == nullptr)
    return;

  // rviz_rendering::MaterialManager::createDefaultColorMaterials()で作成されたマテリアルの重複を防ぐために削除する．
  // TODO: 公式で改善されたらこの処理を削除
  material_manager->remove("RVIZ/Red", "rviz_rendering");
  material_manager->remove("RVIZ/Green", "rviz_rendering");
  material_manager->remove("RVIZ/Blue", "rviz_rendering");
  material_manager->remove("RVIZ/Cyan", "rviz_rendering");
  material_manager->remove("RVIZ/ShadedRed", "rviz_rendering");
  material_manager->remove("RVIZ/ShadedGreen", "rviz_rendering");
  material_manager->remove("RVIZ/ShadedBlue", "rviz_rendering");
  material_manager->remove("RVIZ/ShadedCyan", "rviz_rendering");
}
}  // namespace qt
