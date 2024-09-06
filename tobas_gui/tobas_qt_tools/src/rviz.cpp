#include <rviz_common/yaml_config_reader.hpp>

#include "tobas_qt_tools/rviz.hpp"

namespace qt
{
rviz_common::VisualizationFrame* createRvizFrame(
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
  const QString& config_path,
  QWidget* parent)
{
  // Read configuration
  rviz_common::YamlConfigReader reader;
  rviz_common::Config config;
  reader.readFile(config, config_path);

  // Setup visualization frame
  const auto frame = new rviz_common::VisualizationFrame(rviz_ros_node, parent);
  frame->initialize(rviz_ros_node);
  frame->setHelpPath("");
  frame->setSplashPath("");
  frame->load(config);
  frame->setMenuBar(nullptr);
  frame->setStatusBar(nullptr);
  frame->setHideButtonVisibility(false);
  frame->setStyleSheet("QSizeGrip { width: 0px; height: 0px; }");  // Remove sizegrip

  return frame;
}
}  // namespace qt
