#pragma once

#include <rviz_common/visualization_frame.hpp>

namespace gui
{
namespace core
{
class URDFBuilder : public QWidget
{
  Q_OBJECT

public:
  explicit URDFBuilder(rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_node_if);

private:
  rviz_common::VisualizationFrame* frame_;
};
}  // namespace core
}  // namespace gui
