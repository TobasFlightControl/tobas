#pragma once

#include "./throttles_viewer.hpp"

namespace gui
{
namespace control_system
{
class RCInputViewerWidget : public QWidget
{
  Q_OBJECT

public:
  explicit RCInputViewerWidget(rclcpp::Node::SharedPtr node);

  void updateNamespace(const std::string& ns);

private:
  RCInputThrottlesViewer* throttles_viewer_;
};
}  // namespace control_system
}  // namespace gui
