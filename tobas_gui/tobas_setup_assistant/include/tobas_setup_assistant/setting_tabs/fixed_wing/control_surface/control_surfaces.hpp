#pragma once

#include "../base.hpp"
#include "./available_links.hpp"
#include "./selected_links.hpp"
#include "./add_remove_buttons.hpp"

namespace gui
{
namespace sa
{
namespace fixed_wing
{
class ControlSurfacesWidget : public BaseSelectedLinkSettingWidget
{
  Q_OBJECT

  using self = ControlSurfacesWidget;
  using super = BaseSelectedLinkSettingWidget;

Q_SIGNALS:
  void linkAdded(const QString& link_name);
  void linkRemoved(const QString& link_name);

public:
  explicit ControlSurfacesWidget(const RobotInfo& robot);

  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  const AvailableLinksWidget* available() const;
  const SelectedLinksWidget* selected() const;

  /* 登録された制御面の個数を返す． */
  int numUnits() const;

private:
  AvailableLinksWidget* available_;
  SelectedLinksWidget* selected_;
  AddRemoveButtonsWidget* add_remove_;
};
}  // namespace fixed_wing
}  // namespace sa
}  // namespace gui
