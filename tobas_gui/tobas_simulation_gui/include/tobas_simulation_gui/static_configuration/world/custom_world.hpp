#pragma once

#include <QLineEdit>
#include <QPushButton>

#include <tobas_property_client/property_client.hpp>

#include "./base.hpp"

namespace gui
{
namespace sim
{
class WorldWidget_Custom : public WorldWidget_Base
{
  Q_OBJECT

  using self = WorldWidget_Custom;
  using super = WorldWidget_Base;

  static constexpr char kLastOpenedDirKey[] = "last_opened_dir/custom_world";

public:
  explicit WorldWidget_Custom(rclcpp::Node::SharedPtr node);

  std::filesystem::path worldPath() const override;
  void setContentsEnabled(bool enable) override;

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
