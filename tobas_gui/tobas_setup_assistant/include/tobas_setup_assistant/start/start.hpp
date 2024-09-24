#pragma once

#include "./urdf_loader.hpp"
#include "./package_loader.hpp"

namespace gui
{
namespace setup_assistant
{
class StartWidget : public QWidget
{
  Q_OBJECT

  using self = StartWidget;
  using super = QWidget;

  static constexpr int kHeight = 200;

public:
  explicit StartWidget(rclcpp::Node::SharedPtr node, RobotInfo& robot, SettingsWidget* settings);

private:
  URDFLoaderWidget* urdf_loader_;
  PackageLoaderWidget* package_loader_;
};
};  // namespace setup_assistant
}  // namespace gui
