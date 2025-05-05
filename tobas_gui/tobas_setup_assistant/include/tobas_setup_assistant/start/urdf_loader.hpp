#pragma once

#include <QLineEdit>
#include <QPushButton>
#include <QWidget>

#include <tobas_property_client/property_client.hpp>

#include "../robot_info.hpp"

namespace gui
{
namespace sa
{
class URDFLoaderWidget : public QWidget
{
  Q_OBJECT

  using self = URDFLoaderWidget;
  using super = QWidget;

  static constexpr char kLastOpenedDirKey[] = "last_opened_dir/urdf_loader";

public:
  explicit URDFLoaderWidget(rclcpp::Node::SharedPtr node, RobotInfo& robot);

private Q_SLOTS:
  void onLoadButtonClicked();

private:
  const rclcpp::Node::SharedPtr node_;
  RobotInfo& robot_;

  ptree::PropertyClient property_client_;

  QLineEdit* file_text_;
  QPushButton* load_button_;
};
}  // namespace sa
}  // namespace gui
