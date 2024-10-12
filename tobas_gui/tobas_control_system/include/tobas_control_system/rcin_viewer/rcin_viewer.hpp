#pragma once

#include "./throttles_viewer.hpp"
#include "./toggles_viewer.hpp"

namespace gui
{
namespace control_system
{
namespace rcin
{
class RCInputViewerWidget : public QWidget
{
  Q_OBJECT

public:
  explicit RCInputViewerWidget(rclcpp::Node::SharedPtr node);

  void updateNamespace(const std::string& ns);

private:
  ThrottlesViewer* throttles_viewer_;
  TogglesViewer* toggles_viewer_;
};
}  // namespace rcin
}  // namespace control_system
}  // namespace gui
