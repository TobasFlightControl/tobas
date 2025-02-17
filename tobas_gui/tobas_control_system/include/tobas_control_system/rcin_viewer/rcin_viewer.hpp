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
  explicit RCInputViewerWidget(rclcpp::Node::SharedPtr node);

  void reset();
  void updateNamespace(const std::string& ns);

private:
  ThrottlesViewer* throttles_viewer_;
  TogglesViewer* toggles_viewer_;
};
}  // namespace rcin
}  // namespace gcs
}  // namespace gui
