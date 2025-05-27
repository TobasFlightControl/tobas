#pragma once

#include "./package_loader.hpp"
#include "./urdf_loader.hpp"

namespace gui
{
namespace sa
{
class StartWidget : public QWidget
{
  Q_OBJECT

  using self = StartWidget;
  using super = QWidget;

  static constexpr int kNewId = 0;
  static constexpr int kEditId = kNewId + 1;

public:
  explicit StartWidget(rclcpp::Node::SharedPtr node, RobotInfo& robot, SettingsWidget* settings);

private:
  URDFLoaderWidget* urdf_loader_;
  PackageLoaderWidget* package_loader_;
};
};  // namespace sa
}  // namespace gui
