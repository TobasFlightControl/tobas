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
class URDFLoaderWidget : public QWidget
{
  Q_OBJECT

  using self = URDFLoaderWidget;
  using super = QWidget;

  static constexpr char kLastOpenedDirKey[] = "last_opened_dir/urdf_loader";

public:
  explicit URDFLoaderWidget(rclcpp::Node::SharedPtr node, RobotInfo& robot, SettingsWidget* settings);

private Q_SLOTS:
  void onLoadButtonClicked();

private:
  const rclcpp::Node::SharedPtr node_;
  RobotInfo& robot_;
  SettingsWidget* settings_;

  ptree::PropertyClient property_client_;

  QLineEdit* file_text_;
  QPushButton* load_button_;
};
}  // namespace setup_assistant
}  // namespace gui
