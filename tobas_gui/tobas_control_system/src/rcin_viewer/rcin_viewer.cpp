#include "tobas_control_system/rcin_viewer/rcin_viewer.hpp"

#include <QHBoxLayout>

namespace gui
{
namespace gcs
{
namespace rcin
{
RCInputViewerWidget::RCInputViewerWidget(const RosQtBridge& bridge)
{
  throttles_viewer_ = new ThrottlesViewer(bridge);
  toggles_viewer_ = new TogglesViewer(bridge);

  reset();

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(throttles_viewer_, 3);
  cols->addWidget(toggles_viewer_, 2);
  setLayout(cols);
}

void RCInputViewerWidget::reset()
{
  throttles_viewer_->reset();
  toggles_viewer_->reset();
}
}  // namespace rcin
}  // namespace gcs
}  // namespace gui
