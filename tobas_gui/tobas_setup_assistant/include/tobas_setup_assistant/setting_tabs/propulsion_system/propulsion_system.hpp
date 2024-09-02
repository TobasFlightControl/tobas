#pragma once

#include "../base_setting.hpp"
#include "./available_links.hpp"
#include "./selected_links.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
{
class PropulsionSystemWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = PropulsionSystemWidget;
  using super = BaseSettingWidget;

public:
  explicit PropulsionSystemWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot);

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void onInit() override;
  void onOpened() override;
  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() override;
  void load(const YAML::Node& node) override;

  const AvailableLinksWidget* available() const;
  const SelectedLinksWidget* selected() const;

private Q_SLOTS:
  void onAvailableLinkRemoved(const QString& link_name);
  void onSelectedLinkRemoved(const QString& link_name);

private:
  const rclcpp::Node::SharedPtr node_;
  const RobotInfo& robot_;

  AvailableLinksWidget* available_;
  SelectedLinksWidget* selected_;
};
};  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
