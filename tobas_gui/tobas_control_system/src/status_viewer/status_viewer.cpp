#include <QVBoxLayout>

#include <tobas_qt_tools/widgets/label.hpp>

#include "tobas_control_system/status_viewer/status_viewer.hpp"

namespace gui
{
namespace control_system
{
StatusViewerWidget::StatusViewerWidget(rclcpp::Node::SharedPtr node)
{
  prearm_check_viewer_ = new PreArmCheckViewerWidget(node);
  postarm_check_viewer_ = new PostArmCheckViewerWidget(node);
  other_status_viewer_ = new OtherStatusViewerWidget(node);

  const auto rows = new QVBoxLayout();
  rows->addWidget(new qt::Label("Pre-Arm Check", kLabelPSize, QFont::Bold));
  rows->addWidget(prearm_check_viewer_);
  rows->addWidget(new qt::Label("Post-Arm Check", kLabelPSize, QFont::Bold));
  rows->addWidget(postarm_check_viewer_);
  rows->addWidget(new qt::Label("Other Status", kLabelPSize, QFont::Bold));
  rows->addWidget(other_status_viewer_);
  rows->addStretch();

  setLayout(rows);
}

void StatusViewerWidget::updateNamespace(const std::string& ns)
{
  prearm_check_viewer_->updateNamespace(ns);
  postarm_check_viewer_->updateNamespace(ns);
  other_status_viewer_->updateNamespace(ns);
}
}  // namespace control_system
}  // namespace gui
