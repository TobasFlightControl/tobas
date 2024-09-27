#include <QHBoxLayout>

#include "tobas_control_system/rcin_viewer/rcin_viewer.hpp"

namespace gui
{
namespace control_system
{
namespace rcin
{
RCInputViewerWidget::RCInputViewerWidget(rclcpp::Node::SharedPtr node)
{
  throttles_viewer_ = new ThrottlesViewer(node);
  toggles_viewer_ = new TogglesViewer(node);

  const auto cols = new QHBoxLayout();
  cols->addWidget(throttles_viewer_, 1);
  cols->addWidget(toggles_viewer_, 1);

  setLayout(cols);
}

void RCInputViewerWidget::updateNamespace(const std::string& ns)
{
  throttles_viewer_->updateNamespace(ns);
  toggles_viewer_->updateNamespace(ns);
}
}  // namespace rcin
}  // namespace control_system
}  // namespace gui
