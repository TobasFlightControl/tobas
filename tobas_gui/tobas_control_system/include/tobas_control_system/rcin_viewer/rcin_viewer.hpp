#pragma once

#include "./throttles_viewer.hpp"
#include "./toggles_viewer.hpp"

namespace gui
{
namespace gcs
{
namespace rcin
{
class RCInputViewerWidget : public QWidget
{
  Q_OBJECT

public:
  explicit RCInputViewerWidget(const RosQtBridge& bridge);

  void reset();

private:
  ThrottlesViewer* throttles_viewer_;
  TogglesViewer* toggles_viewer_;
};
}  // namespace rcin
}  // namespace gcs
}  // namespace gui
