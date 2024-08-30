#pragma once

#include <QWidget>
#include <QLineEdit>
#include <QPushButton>

#include <tobas_property_tools/property_client.hpp>

#include "../robot_info.hpp"
#include "../settings.hpp"

namespace gui
{
namespace setup_assistant
{
class PackageLoaderWidget : public QWidget
{
  Q_OBJECT

  using self = PackageLoaderWidget;
  using super = QWidget;

  static constexpr char kLastOpenedDirKey[] = "last_opened_dir/package_loader";

public:
  explicit PackageLoaderWidget(rclcpp::Node::SharedPtr node, RobotInfo& robot, SettingsWidget* settings);

private Q_SLOTS:
  void onLoadButtonClicked();

private:
  rclcpp::Node::SharedPtr node_;
  RobotInfo& robot_;
  SettingsWidget* settings_;

  ptree::PropertyClient property_client_;

  QLineEdit* file_text_;
  QPushButton* load_button_;
};
}  // namespace setup_assistant
}  // namespace gui
