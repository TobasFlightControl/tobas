#include <rviz_common/yaml_config_reader.hpp>

#include "tobas_qt_tools/rviz.hpp"

namespace qt
{
RvizFrameManager::RvizFrameManager()
{
}

void RvizFrameManager::initialize(const std::string& node_name, const QString& config_path, QWidget* parent)
{
  // Initialize ROS node
  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  // Create Rviz ROS interface
  node_ = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(node_name);

  // Read configuration
  rviz_common::YamlConfigReader reader;
  rviz_common::Config config;
  reader.readFile(config, config_path);

  // Setup visualization frame
  frame_ = new rviz_common::VisualizationFrame(node_, parent);
  frame_->initialize(node_);
  frame_->setHelpPath("");
  frame_->setSplashPath("");
  frame_->load(config);
  frame_->setMenuBar(nullptr);
  frame_->setStatusBar(nullptr);
  frame_->setHideButtonVisibility(false);
  frame_->setStyleSheet("QSizeGrip { width: 0px; height: 0px; }");  // Remove sizegrip
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

rviz_common::VisualizationFrame* RvizFrameManager::frame()
{
  return frame_;
}
}  // namespace qt
