#pragma once

#include "../base_setting.hpp"
#include "./available_links.hpp"
#include "./selected_links.hpp"
#include "./add_remove_buttons.hpp"

namespace gui
{
namespace setup_assistant
{
namespace servo_joint
{
class ServoJointsWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = ServoJointsWidget;
  using super = BaseSettingWidget;

public:
  explicit ServoJointsWidget(const RobotInfo& robot);

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void onOpened() override;
  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() override;
  void load(const YAML::Node& node) override;

  const AvailableLinksWidget* available() const;
  const SelectedLinksWidget* selected() const;

private:
  const RobotInfo& robot_;

  AvailableLinksWidget* available_;
  SelectedLinksWidget* selected_;
  AddRemoveButtonsWidget* add_remove_;
};
}  // namespace servo_joint
}  // namespace setup_assistant
}  // namespace gui
