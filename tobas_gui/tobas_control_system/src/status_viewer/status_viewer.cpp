#include "tobas_control_system/status_viewer/status_viewer.hpp"

#include <QVBoxLayout>

#include <tobas_qt_tools/widgets/label.hpp>

namespace gui
{
namespace gcs
{
StatusViewerWidget::StatusViewerWidget(const RosQtBridge& bridge)
{
  prearm_check_viewer_ = new PreArmCheckViewerWidget(bridge);
  postarm_check_viewer_ = new PostArmCheckViewerWidget(bridge);
  other_status_viewer_ = new OtherStatusViewerWidget(bridge);

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

void StatusViewerWidget::reset()
{
  prearm_check_viewer_->reset();
  postarm_check_viewer_->reset();
  other_status_viewer_->reset();
}
}  // namespace gcs
}  // namespace gui
