#include <QHBoxLayout>

#include "tobas_control_system/rcin_viewer/rcin_viewer.hpp"

namespace gui
{
namespace control_system
{
RCInputViewerWidget::RCInputViewerWidget(rclcpp::Node::SharedPtr node)
{
  throttles_viewer_ = new RCInputThrottlesViewer(node);

  const auto cols = new QHBoxLayout();
  cols->addWidget(throttles_viewer_);
  // TODO

  setLayout(cols);
}

void RCInputViewerWidget::updateNamespace(const std::string& ns)
{
  throttles_viewer_->updateNamespace(ns);
}
}  // namespace control_system
}  // namespace gui
