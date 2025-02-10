#pragma once

#include "./urdf_loader.hpp"
#include "./package_loader.hpp"

namespace gui
{
namespace sa
{
class StartWidget : public QWidget
{
  Q_OBJECT

  using self = StartWidget;
  using super = QWidget;

public:
  explicit StartWidget(rclcpp::Node::SharedPtr node, RobotInfo& robot, SettingsWidget* settings);

private:
  URDFLoaderWidget* urdf_loader_;
  PackageLoaderWidget* package_loader_;
};
};  // namespace sa
}  // namespace gui
