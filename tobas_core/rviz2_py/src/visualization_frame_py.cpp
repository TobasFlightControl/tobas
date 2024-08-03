#include "../include/rviz2_py/visualization_frame_py.hpp"

namespace rviz_common
{
VisualizationFramePy::VisualizationFramePy(QWidget* parent)
  : super(ros_integration::RosNodeAbstractionIface::WeakPtr(), parent)
{
  rviz_ros_node_ = client_.init(0, nullptr, "rviz", false);
}

VisualizationFramePy::~VisualizationFramePy()
{
  client_.shutdown();
}

void VisualizationFramePy::initialize(const QString& display_config_file)
{
  VisualizationFrame::initialize(rviz_ros_node_, display_config_file);
}
}  // namespace rviz_common
