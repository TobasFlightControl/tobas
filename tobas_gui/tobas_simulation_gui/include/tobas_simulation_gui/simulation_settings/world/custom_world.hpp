#pragma once

#include <QLineEdit>
#include <QPushButton>

#include <tobas_property_client/property_client.hpp>

#include "./base.hpp"

namespace gui
{
namespace sim
{
class CustomWorldWidget : public BaseWorldWidget
{
  Q_OBJECT

  using self = CustomWorldWidget;
  using super = BaseWorldWidget;

  static constexpr char kLastOpenedDirKey[] = "last_opened_dir";

public:
  explicit CustomWorldWidget(rclcpp::Node::SharedPtr node);

  std::filesystem::path worldPath() const override;

private:
  const rclcpp::Node::SharedPtr node_;
  ptree::PropertyClient property_client_;

  QLineEdit* file_text_;
  QPushButton* browse_button_;

private Q_SLOTS:
  void onBrowseButtonClicked();
};
}  // namespace sim
}  // namespace gui
