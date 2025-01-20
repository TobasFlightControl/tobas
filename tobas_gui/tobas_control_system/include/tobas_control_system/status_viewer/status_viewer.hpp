#pragma once

#include <tobas_qt_tools/widgets/scroll_area.hpp>

#include "./prearm_check_viewer.hpp"
#include "./postarm_check_viewer.hpp"
#include "./other_status_viewer.hpp"

namespace gui
{
namespace control_system
{
class StatusViewerWidget : public qt::ScrollArea
{
  Q_OBJECT

  using self = StatusViewerWidget;
  using super = qt::ScrollArea;

  static constexpr int kLabelPSize = 18;

public:
  explicit StatusViewerWidget(rclcpp::Node::SharedPtr node);

  void updateNamespace(const std::string& ns);

private:
  PreArmCheckViewerWidget* prearm_check_viewer_;
  PostArmCheckViewerWidget* postarm_check_viewer_;
  OtherStatusViewerWidget* other_status_viewer_;
};
}  // namespace control_system
}  // namespace gui
